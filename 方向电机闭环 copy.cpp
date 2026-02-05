#include <gpiod.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// ================= PWM 配置（sysfs PWM） =================
const std::string PWM_CHIP_PATH = "/sys/class/pwm/pwmchip0/";
const int PWM_CHANNEL = 1;               // GPIO19 -> PWM1 通道（取决于你的系统映射）
const long PWM_PERIOD_NS = 100000;       // 10kHz
// =========================================================

// ================= 方向 GPIO 配置（libgpiod） =============
const int DIR_GPIO_OFFSET = 16;          // GPIO21 控制正反转（BCM21）
const int MAX_GPIOCHIP = 2;
// =========================================================

// ================= 编码器 GPIO 配置 =======================
#define HU_LINE_OFFSET 5
#define HV_LINE_OFFSET 6
#define HW_LINE_OFFSET 13
// =========================================================

// ================= 编码器参数 =============================
#define ENCODER_COUNTS_PER_REV 5770
#define EVENT_DEBOUNCE_US 100
// =========================================================

// ================= 控制参数 ===============================
const double CONTROL_HZ = 50.0;
const double KP = 1.2;
const double KI = 0.0;
const double KD = 0.0;
const double I_MAX = 50.0;
const double DEAD_BAND_DEG = 0.5;
const double MAX_PWM_PERCENT = 70.0;
const double MAX_PWM_STEP = 2.0;
const bool INVERT_ENCODER = true;
// =========================================================

static const char *forward_codes[] = {"100", "110", "010", "011", "001", "101"};
#define CODE_COUNT 6

static std::atomic<bool> exit_flag{false};
static std::atomic<double> target_angle_deg{0.0};

// ---------------- 时间工具 ----------------
static unsigned long long micros64() {
    using namespace std::chrono;
    return (unsigned long long)duration_cast<microseconds>(
               steady_clock::now().time_since_epoch())
        .count();
}

// ---------------- PWM sysfs ----------------
static bool writeToFile(const std::string &path, const std::string &value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << path << "\n";
        return false;
    }
    file << value;
    file.close();
    return true;
}

static bool initPWM() {
    std::string exportPath = PWM_CHIP_PATH + "export";
    (void)writeToFile(exportPath, std::to_string(PWM_CHANNEL));

    std::string periodPath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/period";
    if (!writeToFile(periodPath, std::to_string(PWM_PERIOD_NS))) return false;

    std::string dutyPath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
    if (!writeToFile(dutyPath, "0")) return false;

    std::string enablePath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/enable";
    if (!writeToFile(enablePath, "1")) return false;

    std::cout << "PWM 初始化完成（pwmchip0/pwm" << PWM_CHANNEL << "）\n";
    return true;
}

static bool setPWMDutyCycle(double dutyPercent) {
    if (dutyPercent < 0) dutyPercent = 0;
    if (dutyPercent > 100) dutyPercent = 100;

    long dutyNs = (long)std::llround((double)PWM_PERIOD_NS * (dutyPercent / 100.0));
    std::string dutyPath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
    return writeToFile(dutyPath, std::to_string(dutyNs));
}

static void closePWM() {
    std::string enablePath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/enable";
    (void)writeToFile(enablePath, "0");
    std::string unexportPath = PWM_CHIP_PATH + "unexport";
    (void)writeToFile(unexportPath, std::to_string(PWM_CHANNEL));
}

// ---------------- 方向 GPIO ----------------
static char *detect_gpiochip_path(char out[32]) {
    struct stat st;
    for (int i = 0; i < MAX_GPIOCHIP; i++) {
        char p[32];
        snprintf(p, sizeof(p), "/dev/gpiochip%d", i);
        if (stat(p, &st) == 0 && S_ISCHR(st.st_mode)) {
            int fd = open(p, O_RDONLY);
            if (fd >= 0) {
                close(fd);
                strncpy(out, p, 31);
                out[31] = '\0';
                return out;
            }
        }
    }
    return nullptr;
}

struct DirGpio {
    gpiod_chip *chip = nullptr;
    gpiod_line *line = nullptr;
};

static bool initDirectionGPIO(DirGpio &dir) {
    char chip_path[32] = {0};
    if (!detect_gpiochip_path(chip_path)) {
        std::cerr << "未检测到 /dev/gpiochipX\n";
        return false;
    }

    dir.chip = gpiod_chip_open(chip_path);
    if (!dir.chip) {
        perror("打开GPIO芯片失败");
        return false;
    }

    dir.line = gpiod_chip_get_line(dir.chip, DIR_GPIO_OFFSET);
    if (!dir.line) {
        perror("获取方向GPIO线失败");
        gpiod_chip_close(dir.chip);
        dir.chip = nullptr;
        return false;
    }

    if (gpiod_line_request_output(dir.line, "motor-dir", 0) != 0) {
        perror("请求方向GPIO输出失败");
        gpiod_chip_close(dir.chip);
        dir.chip = nullptr;
        dir.line = nullptr;
        return false;
    }
    return true;
}

static bool setDirection(DirGpio &dir, bool forward) {
    if (!dir.line) return false;
    int val = forward ? 1 : 0;
    return gpiod_line_set_value(dir.line, val) == 0;
}

static void closeDirectionGPIO(DirGpio &dir) {
    if (dir.line) {
        gpiod_line_release(dir.line);
        dir.line = nullptr;
    }
    if (dir.chip) {
        gpiod_chip_close(dir.chip);
        dir.chip = nullptr;
    }
}

// ---------------- 编码器 ----------------
static gpiod_chip *chip = nullptr;
static gpiod_line *hu_line = nullptr;
static gpiod_line *hv_line = nullptr;
static gpiod_line *hw_line = nullptr;
static char gpiochip_path[32] = {0};

static int hu_level = 0, hv_level = 0, hw_level = 0;
static unsigned long long last_evt_us_hu = 0;
static unsigned long long last_evt_us_hv = 0;
static unsigned long long last_evt_us_hw = 0;

static long total_step = 0;
static char last_code[4] = "000";

static int is_valid_hall_code(const char *code) {
    for (int i = 0; i < CODE_COUNT; i++) {
        if (strcmp(code, forward_codes[i]) == 0) return 1;
    }
    return 0;
}

static int get_code_index(const char *code) {
    for (int i = 0; i < CODE_COUNT; i++) {
        if (strcmp(code, forward_codes[i]) == 0) return i;
    }
    return -1;
}

static void make_code(char out[4]) {
    snprintf(out, 4, "%d%d%d", hu_level, hv_level, hw_level);
}

static void update_encoder(const char *current_code) {
    if (!is_valid_hall_code(current_code)) return;

    if (strcmp(last_code, "000") == 0) {
        strncpy(last_code, current_code, 3);
        last_code[3] = '\0';
        return;
    }

    if (strcmp(current_code, last_code) == 0) return;

    int curr_idx = get_code_index(current_code);
    int last_idx = get_code_index(last_code);
    if (curr_idx < 0 || last_idx < 0) return;

    if ((last_idx + 1) % CODE_COUNT == curr_idx) {
        total_step++;
        strncpy(last_code, current_code, 3);
        last_code[3] = '\0';
        return;
    }

    if ((last_idx - 1 + CODE_COUNT) % CODE_COUNT == curr_idx) {
        total_step--;
        strncpy(last_code, current_code, 3);
        last_code[3] = '\0';
        return;
    }

    strncpy(last_code, current_code, 3);
    last_code[3] = '\0';
}

static void process_one_event(gpiod_line *line,
                              unsigned long long *plast_us,
                              int *plevel) {
    gpiod_line_event ev;
    if (gpiod_line_event_read(line, &ev) < 0) return;

    unsigned long long now_us = micros64();
    if (*plast_us != 0 && (now_us - *plast_us) < (unsigned long long)EVENT_DEBOUNCE_US) {
        return;
    }
    *plast_us = now_us;

    if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) *plevel = 1;
    else if (ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) *plevel = 0;

    char code[4];
    make_code(code);
    update_encoder(code);
}

static int gpio_init_irq() {
    if (detect_gpiochip_path(gpiochip_path) == nullptr) {
        std::cerr << "错误：未检测到GPIO芯片（/dev/gpiochipX）\n";
        return -1;
    }

    chip = gpiod_chip_open(gpiochip_path);
    if (!chip) {
        perror("打开GPIO芯片失败");
        return -1;
    }

    hu_line = gpiod_chip_get_line(chip, HU_LINE_OFFSET);
    hv_line = gpiod_chip_get_line(chip, HV_LINE_OFFSET);
    hw_line = gpiod_chip_get_line(chip, HW_LINE_OFFSET);
    if (!hu_line || !hv_line || !hw_line) {
        perror("获取GPIO线失败");
        return -1;
    }

    const char *consumer = "hall-encoder-irq";
    if (gpiod_line_request_both_edges_events(hu_line, consumer) ||
        gpiod_line_request_both_edges_events(hv_line, consumer) ||
        gpiod_line_request_both_edges_events(hw_line, consumer)) {
        perror("请求GPIO事件模式失败（权限/占用？）");
        return -1;
    }

    int v;
    v = gpiod_line_get_value(hu_line); hu_level = (v < 0) ? 0 : v;
    v = gpiod_line_get_value(hv_line); hv_level = (v < 0) ? 0 : v;
    v = gpiod_line_get_value(hw_line); hw_level = (v < 0) ? 0 : v;
    return 0;
}

static void gpio_deinit() {
    if (hu_line) { gpiod_line_release(hu_line); hu_line = nullptr; }
    if (hv_line) { gpiod_line_release(hv_line); hv_line = nullptr; }
    if (hw_line) { gpiod_line_release(hw_line); hw_line = nullptr; }
    if (chip) { gpiod_chip_close(chip); chip = nullptr; }
}

// ---------------- 控制逻辑 ----------------
static double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void sig_handler(int) {
    exit_flag.store(true, std::memory_order_relaxed);
}

static void input_thread() {
    std::string line;
    while (!exit_flag.load(std::memory_order_relaxed)) {
        std::cout << "\n输入目标角度(deg)，回车确认：";
        if (!std::getline(std::cin, line)) break;
        try {
            double val = std::stod(line);
            target_angle_deg.store(val, std::memory_order_relaxed);
            std::cout << "目标角度更新为 " << val << " deg\n";
        } catch (...) {
            std::cout << "输入无效\n";
        }
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, sig_handler);

    if (argc >= 2) {
        target_angle_deg.store(std::stod(argv[1]), std::memory_order_relaxed);
    }

    if (!initPWM()) {
        std::cerr << "PWM初始化失败\n";
        return 1;
    }

    DirGpio dir;
    if (!initDirectionGPIO(dir)) {
        std::cerr << "方向GPIO初始化失败\n";
        closePWM();
        return 1;
    }

    if (gpio_init_irq() != 0) {
        std::cerr << "编码器GPIO初始化失败\n";
        closeDirectionGPIO(dir);
        closePWM();
        return 1;
    }

    int fd_hu = gpiod_line_event_get_fd(hu_line);
    int fd_hv = gpiod_line_event_get_fd(hv_line);
    int fd_hw = gpiod_line_event_get_fd(hw_line);
    if (fd_hu < 0 || fd_hv < 0 || fd_hw < 0) {
        std::cerr << "获取事件fd失败\n";
        gpio_deinit();
        closeDirectionGPIO(dir);
        closePWM();
        return 1;
    }

    pollfd fds[3];
    fds[0].fd = fd_hu; fds[0].events = POLLIN;
    fds[1].fd = fd_hv; fds[1].events = POLLIN;
    fds[2].fd = fd_hw; fds[2].events = POLLIN;

    std::thread input_thr(input_thread);

    const double steps_per_rev = (double)ENCODER_COUNTS_PER_REV;
    const double deg_per_step = 360.0 / steps_per_rev;

    double integral = 0.0;
    double prev_error = 0.0;
    double prev_output = 0.0;

    auto last_time = std::chrono::steady_clock::now();
    auto next_control = last_time;
    const auto control_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / CONTROL_HZ));

    while (!exit_flag.load(std::memory_order_relaxed)) {
        int ret = poll(fds, 3, 5);
        if (ret > 0) {
            if (fds[0].revents & POLLIN) process_one_event(hu_line, &last_evt_us_hu, &hu_level);
            if (fds[1].revents & POLLIN) process_one_event(hv_line, &last_evt_us_hv, &hv_level);
            if (fds[2].revents & POLLIN) process_one_event(hw_line, &last_evt_us_hw, &hw_level);
        } else if (ret < 0 && errno != EINTR) {
            perror("poll失败");
            break;
        }

        auto now = std::chrono::steady_clock::now();
        if (now < next_control) continue;
        next_control = now + control_period;

        double angle = (double)(INVERT_ENCODER ? -total_step : total_step) * deg_per_step;
        double target = target_angle_deg.load(std::memory_order_relaxed);
        double error = target - angle;
        if (std::abs(error) < DEAD_BAND_DEG) error = 0.0;

        double dt = std::chrono::duration<double>(now - last_time).count();
        if (dt <= 0.0) continue;
        last_time = now;

        integral += error * dt;
        integral = clamp(integral, -I_MAX, I_MAX);
        double derivative = (error - prev_error) / dt;
        prev_error = error;

        double raw_output = (KP * error) + (KI * integral) + (KD * derivative);
        double output = clamp(raw_output, -MAX_PWM_PERCENT, MAX_PWM_PERCENT);

        double delta = output - prev_output;
        if (delta > MAX_PWM_STEP) output = prev_output + MAX_PWM_STEP;
        else if (delta < -MAX_PWM_STEP) output = prev_output - MAX_PWM_STEP;
        prev_output = output;

        bool forward = output >= 0.0;
        if (!setDirection(dir, forward)) {
            std::cerr << "设置方向失败\n";
            break;
        }

        double duty = std::abs(output);
        if (!setPWMDutyCycle(duty)) {
            std::cerr << "设置PWM失败\n";
            break;
        }

        std::cout << "\rangle=" << angle
                  << " target=" << target
                  << " err=" << error
                  << " out%=" << output
                  << "   " << std::flush;
    }

    exit_flag.store(true, std::memory_order_relaxed);
    if (input_thr.joinable()) input_thr.join();

    setPWMDutyCycle(0);
    closePWM();
    closeDirectionGPIO(dir);
    gpio_deinit();
    std::cout << "\n退出\n";
    return 0;
}
