#include <gpiod.h>

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cmath>
#include <cstring>      // strncpy

#include <unistd.h>     // close, usleep, sleep (以及部分系统的 open)
#include <fcntl.h>      // O_RDONLY
#include <sys/stat.h>   // struct stat, stat(), S_ISCHR


// ================= PWM 配置（sysfs PWM） =================
const std::string PWM_CHIP_PATH = "/sys/class/pwm/pwmchip0/";
const int PWM_CHANNEL = 1;               // GPIO19 -> PWM1 通道（取决于你的系统映射）
const long PWM_PERIOD_NS = 100000;       // 10kHz
const float MAX_VOLTAGE = 3.3f;
// =========================================================

// ================= 方向 GPIO 配置（libgpiod） =============
const int DIR_GPIO_OFFSET = 16;          // GPIO21 控制正反转（BCM21）
const int MAX_GPIOCHIP = 2;
// =========================================================

// -------- sysfs 写文件 --------
static bool writeToFile(const std::string& path, const std::string& value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << path << "\n";
        return false;
    }
    file << value;
    file.close();
    return true;
}

// -------- PWM 控制 --------
static bool initPWM() {
    // export（已导出会失败：允许失败）
    std::string exportPath = PWM_CHIP_PATH + "export";
    (void)writeToFile(exportPath, std::to_string(PWM_CHANNEL));

    // 设置周期
    std::string periodPath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/period";
    if (!writeToFile(periodPath, std::to_string(PWM_PERIOD_NS))) return false;

    // duty=0
    std::string dutyPath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";
    if (!writeToFile(dutyPath, "0")) return false;

    // enable=1
    std::string enablePath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/enable";
    if (!writeToFile(enablePath, "1")) return false;

    std::cout << "PWM 初始化完成（pwmchip0/pwm" << PWM_CHANNEL << "，10kHz）\n";
    return true;
}

static bool setPWMDutyCycle(float dutyPercent) {
    if (dutyPercent < 0) dutyPercent = 0;
    if (dutyPercent > 100) dutyPercent = 100;

    long dutyNs = (long)std::llround((double)PWM_PERIOD_NS * (dutyPercent / 100.0));
    std::string dutyPath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/duty_cycle";

    if (!writeToFile(dutyPath, std::to_string(dutyNs))) return false;

    float voltage = (dutyPercent / 100.0f) * MAX_VOLTAGE;
    std::cout << "占空比=" << dutyPercent << "% | 约等效电压=" << voltage << "V\n";
    return true;
}

static void closePWM() {
    std::string enablePath = PWM_CHIP_PATH + "pwm" + std::to_string(PWM_CHANNEL) + "/enable";
    (void)writeToFile(enablePath, "0");

    std::string unexportPath = PWM_CHIP_PATH + "unexport";
    (void)writeToFile(unexportPath, std::to_string(PWM_CHANNEL));

    std::cout << "PWM 已关闭并释放\n";
}

// -------- gpiod 方向控制 --------
static char* detect_gpiochip_path(char out[32]) {
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
    gpiod_chip* chip = nullptr;
    gpiod_line* line = nullptr;
};

static bool initDirectionGPIO(DirGpio& dir) {
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

    std::cout << "方向 GPIO 初始化完成（offset=" << DIR_GPIO_OFFSET << "）\n";
    return true;
}

static void setDirection(DirGpio& dir, bool forward) {
    if (!dir.line) throw std::runtime_error("方向GPIO未初始化");
    int val = forward ? 1 : 0;
    if (gpiod_line_set_value(dir.line, val) != 0) {
        throw std::runtime_error("设置方向GPIO失败");
    }
    std::cout << "方向=" << (forward ? "正转" : "反转") << " (GPIO21=" << val << ")\n";
}

static void closeDirectionGPIO(DirGpio& dir) {
    if (dir.line) {
        gpiod_line_release(dir.line);
        dir.line = nullptr;
    }
    if (dir.chip) {
        gpiod_chip_close(dir.chip);
        dir.chip = nullptr;
    }
    std::cout << "方向 GPIO 已释放\n";
}

// ---------------- main ----------------
int main() {
    DirGpio dir;

    try {
        if (!initPWM()) throw std::runtime_error("PWM初始化失败");
        if (!initDirectionGPIO(dir)) throw std::runtime_error("方向GPIO初始化失败");

        std::cout << "开始输出 PWM(GPIO19) + 方向(GPIO21)...\n";

        // 正转 50%
        setDirection(dir, false);
        setPWMDutyCycle(50);
        sleep(3);

        // 反转 50%
        setDirection(dir, true);
        setPWMDutyCycle(50);
        sleep(3);

        // 停止
        setPWMDutyCycle(0);
        sleep(1);

        closePWM();
        closeDirectionGPIO(dir);
    } catch (const std::exception& e) {
        std::cerr << "程序异常：" << e.what() << "\n";
        // 出错也尽量释放
        setPWMDutyCycle(0);
        closePWM();
        closeDirectionGPIO(dir);
        return 1;
    }

    return 0;
}
