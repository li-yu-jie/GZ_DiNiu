#include <gpiod.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <signal.h>
#include <string>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

// ==================== 配置 ====================
#define HU_LINE_OFFSET 5
#define HV_LINE_OFFSET 6
#define HW_LINE_OFFSET 13

#define SAMPLE_INTERVAL_MS 100
#define MAX_GPIOCHIP 2

#define P 4
#define SPEED_CALC_MS 1000
#define HALL_STEP_PER_CYCLE 6

#define STOP_TIMEOUT_MS 200
#define EVENT_DEBOUNCE_US 100    // 同一线两次事件间隔<100us忽略（防抖，可调）

// 打开/关闭事件调试打印（1=开，0=关）
#define DEBUG_EVENT 1

static const char *forward_codes[] = {"100", "110", "010", "011", "001", "101"};
#define CODE_COUNT 6
// =============================================

// 全局资源（保持和原逻辑一致）
static gpiod_chip *chip = nullptr;
static gpiod_line *hu_line = nullptr;
static gpiod_line *hv_line = nullptr;
static gpiod_line *hw_line = nullptr;
static char gpiochip_path[32] = {0};

// 退出标志：信号安全
static std::atomic<bool> exit_flag{false};

// 状态锁
static std::mutex state_lock;

// 状态
static long encoder_step = 0;
static long total_step = 0;
static int motor_direction = 0;   // 0静止 1正转 -1反转
static float motor_speed = 0.0f;

static char last_code[4] = "000";
static unsigned long last_step_ms = 0;

static long last_total_step = 0;
static unsigned long speed_calc_start = 0;

// 当前三路电平（关键：用事件更新，不重采样）
static int hu_level = 0, hv_level = 0, hw_level = 0;

// 每条线消抖：最后一次事件时间(us)
static unsigned long long last_evt_us_hu = 0;
static unsigned long long last_evt_us_hv = 0;
static unsigned long long last_evt_us_hw = 0;

// ---------------- 时间工具（C++ chrono） ----------------
static unsigned long millis() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static unsigned long long micros64() {
    using namespace std::chrono;
    return (unsigned long long)duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

// 信号处理：只置标志
static void sig_handler(int) {
    exit_flag.store(true, std::memory_order_relaxed);
}

// ---------------- GPIO chip 探测 ----------------
static int detect_gpiochip(char *out, size_t outlen) {
    struct stat st;
    for (int i = 0; i < MAX_GPIOCHIP; i++) {
        char p[32];
        snprintf(p, sizeof(p), "/dev/gpiochip%d", i);
        if (stat(p, &st) == 0 && S_ISCHR(st.st_mode)) {
            int fd = open(p, O_RDONLY);
            if (fd >= 0) {
                close(fd);
                strncpy(out, p, outlen - 1);
                out[outlen - 1] = '\0';
                return 0;
            }
        }
    }
    return -1;
}

// ---------------- 编码工具 ----------------
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

// 计数更新（必须在 lock 内）
static void update_encoder_locked(const char *current_code, unsigned long now_ms) {
    if (!is_valid_hall_code(current_code)) {
        return;
    }

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
        motor_direction = 1;
        encoder_step++;
        total_step++;
        last_step_ms = now_ms;
        strncpy(last_code, current_code, 3);
        last_code[3] = '\0';
        return;
    }

    if ((last_idx - 1 + CODE_COUNT) % CODE_COUNT == curr_idx) {
        motor_direction = -1;
        encoder_step--;
        total_step--;
        last_step_ms = now_ms;
        strncpy(last_code, current_code, 3);
        last_code[3] = '\0';
        return;
    }

    // 跳码：只同步不计数
    strncpy(last_code, current_code, 3);
    last_code[3] = '\0';
}

static void calc_real_speed_locked(unsigned long now_ms) {
    if (speed_calc_start == 0) {
        speed_calc_start = now_ms;
        last_total_step = total_step;
        return;
    }

    unsigned long dt = now_ms - speed_calc_start;
    if (dt >= (unsigned long)SPEED_CALC_MS) {
        long diff = total_step - last_total_step;
        long step_diff = labs(diff);

        if (step_diff > 0) {
            motor_speed = (float)((60.0 * (double)step_diff) /
                                  (double)(P * HALL_STEP_PER_CYCLE) /
                                  ((double)dt / 1000.0));
        } else {
            motor_speed = 0.0f;
        }

        last_total_step = total_step;
        speed_calc_start = now_ms;
    }

    if (last_step_ms != 0 && (now_ms - last_step_ms) >= (unsigned long)STOP_TIMEOUT_MS) {
        motor_direction = 0;
        motor_speed = 0.0f;
    }
}

// ---------------- GPIO init/deinit ----------------
static int gpio_init_irq() {
    if (detect_gpiochip(gpiochip_path, sizeof(gpiochip_path)) != 0) {
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

    std::cout << "GPIO " << HU_LINE_OFFSET << "/" << HV_LINE_OFFSET << "/" << HW_LINE_OFFSET
              << " 初始化完成（事件模式），chip=" << gpiochip_path << "\n";

#if DEBUG_EVENT
    std::cout << "[DEBUG] 初始电平 HU=" << hu_level << " HV=" << hv_level << " HW=" << hw_level
              << " -> code=" << hu_level << hv_level << hw_level << "\n";
#endif
    return 0;
}

static void gpio_deinit() {
    if (hu_line) { gpiod_line_release(hu_line); hu_line = nullptr; }
    if (hv_line) { gpiod_line_release(hv_line); hv_line = nullptr; }
    if (hw_line) { gpiod_line_release(hw_line); hw_line = nullptr; }
    if (chip) { gpiod_chip_close(chip); chip = nullptr; }
}

// ---------------- 事件处理：每次只读一个事件 ----------------
static void process_one_event(gpiod_line *line,
                              const char *name,
                              unsigned long long *plast_us,
                              int *plevel) {
    gpiod_line_event ev;
    if (gpiod_line_event_read(line, &ev) < 0) return;

    unsigned long long now_us = micros64();
    if (*plast_us != 0 && (now_us - *plast_us) < (unsigned long long)EVENT_DEBOUNCE_US) {
#if DEBUG_EVENT
        std::cout << "\n[DEBUG] " << name << " event ignored (debounce)\n";
#endif
        return;
    }
    *plast_us = now_us;

    if (ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) *plevel = 1;
    else if (ev.event_type == GPIOD_LINE_EVENT_FALLING_EDGE) *plevel = 0;

    char code[4];
    make_code(code);

    unsigned long now_ms = (unsigned long)(now_us / 1000ULL);

    {
        std::lock_guard<std::mutex> lg(state_lock);
        update_encoder_locked(code, now_ms);
        calc_real_speed_locked(now_ms);
    }

#if DEBUG_EVENT
    std::cout << "\n[DEBUG] " << name << " "
              << ((ev.event_type == GPIOD_LINE_EVENT_RISING_EDGE) ? "RISING" : "FALLING")
              << " -> HU=" << hu_level << " HV=" << hv_level << " HW=" << hw_level
              << " code=" << code << " (valid=" << is_valid_hall_code(code) << ")\n";
#endif
}

// ---------------- main ----------------
int main() {
    signal(SIGINT, sig_handler);

    if (gpio_init_irq() != 0) {
        std::cerr << "GPIO初始化失败，程序退出\n";
        gpio_deinit();
        return -1;
    }

    int fd_hu = gpiod_line_event_get_fd(hu_line);
    int fd_hv = gpiod_line_event_get_fd(hv_line);
    int fd_hw = gpiod_line_event_get_fd(hw_line);
    if (fd_hu < 0 || fd_hv < 0 || fd_hw < 0) {
        std::cerr << "获取事件fd失败（libgpiod版本/权限问题？）\n";
        gpio_deinit();
        return -1;
    }

    pollfd fds[3];
    fds[0].fd = fd_hu; fds[0].events = POLLIN;
    fds[1].fd = fd_hv; fds[1].events = POLLIN;
    fds[2].fd = fd_hw; fds[2].events = POLLIN;

    std::cout << "\n===== 霍尔编码器（中断/事件模式，P=" << P << "）=====\n";
    std::cout << "配置：速度窗口=" << SPEED_CALC_MS << "ms | 判停超时=" << STOP_TIMEOUT_MS
              << "ms | 事件消抖=" << EVENT_DEBOUNCE_US << "us\n";
    std::cout << "=================================================\n";
    std::cout << "编码\t有效性\t步长计数\t方向\t实际转速(r/min)\n";
    std::cout << "=================================================\n";

    // 启动时同步 last_code（如果有效）
    {
        char init_code[4];
        make_code(init_code);
        std::lock_guard<std::mutex> lg(state_lock);
        if (is_valid_hall_code(init_code)) {
            strncpy(last_code, init_code, 3);
            last_code[3] = '\0';
        }
        last_step_ms = millis();
    }

    unsigned long last_print = millis();

    while (!exit_flag.load(std::memory_order_relaxed)) {
        int ret = poll(fds, 3, 20);

        unsigned long now_ms = millis();
        {
            std::lock_guard<std::mutex> lg(state_lock);
            calc_real_speed_locked(now_ms);
        }

        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("poll失败");
            break;
        }

        if (ret > 0) {
            if (fds[0].revents & POLLIN) process_one_event(hu_line, "HU", &last_evt_us_hu, &hu_level);
            if (fds[1].revents & POLLIN) process_one_event(hv_line, "HV", &last_evt_us_hv, &hv_level);
            if (fds[2].revents & POLLIN) process_one_event(hw_line, "HW", &last_evt_us_hw, &hw_level);
        }

        if ((millis() - last_print) >= (unsigned long)SAMPLE_INTERVAL_MS) {
            char code_now[4];
            make_code(code_now);
            int valid = is_valid_hall_code(code_now);

            long step;
            int dir;
            float spd;
            {
                std::lock_guard<std::mutex> lg(state_lock);
                step = encoder_step;
                dir  = motor_direction;
                spd  = motor_speed;
            }

            std::cout << code_now << "\t" << (valid ? "有效" : "无效") << "\t"
                      << step << "\t\t"
                      << (dir == 1 ? "正转" : (dir == -1 ? "反转" : "静止")) << "\t"
                      << spd << "\r" << std::flush;

            last_print = millis();
        }
    }

    std::cout << "\n\n===== 退出 =====\n";
    {
        std::lock_guard<std::mutex> lg(state_lock);
        std::cout << "最终状态：步长计数=" << encoder_step
                  << " | 方向=" << (motor_direction == 1 ? "正转" : (motor_direction == -1 ? "反转" : "静止"))
                  << " | 转速=" << motor_speed << " r/min\n";
    }

    gpio_deinit();
    return 0;
}
