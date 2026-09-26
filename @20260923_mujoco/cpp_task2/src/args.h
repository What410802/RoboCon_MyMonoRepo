// 共用的命令行解析：认不出的选项、缺值、多余的位置参数、非数字的数值参数都在这里报错退出，
// 主程序只干两件事——声明自己有哪些选项（kOptionDefs）、取值。
//
// 退出码约定：0 = 正常（含 --help）、1 = 参数错误、2 = 判定“未静止”（由调用方自己用）。
#pragma once

#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

// 一个选项：名字（带 --）与是否带值（开关型如 --no-record 不带值）
struct OptionDef {
    const char *name;
    bool takes_value;
};

struct Args {
    std::vector<std::string> positional;        // 位置参数，按出现顺序
    std::map<std::string, std::string> options; // 出现过的选项：名字 → 值（开关的值为空串）

    bool has(const char *name) const { return options.count(name) > 0; }

    std::string value(const char *name, const std::string &fallback = "") const {
        const auto it = options.find(name);
        return it == options.end() ? fallback : it->second;
    }

    double number(const char *name, double fallback, const char *usage) const {
        const auto it = options.find(name);
        return it == options.end() ? fallback : ParseNum(it->second, name, usage);
    }

    int integer(const char *name, int fallback, const char *usage) const {
        return static_cast<int>(number(name, fallback, usage));
    }

    // text 必须正好是一个数字，否则报错退出
    static double ParseNum(const std::string &text, const char *name, const char *usage) {
        const char *begin = text.c_str();
        char *end = nullptr;
        const double v = std::strtod(begin, &end);
        if (end == begin || *end != '\0') {
            std::fprintf(stderr, "%s 不是数字：%s\n%s", name, text.c_str(), usage);
            std::exit(1);
        }
        return v;
    }
};

// 解析 argv：--help/-h 打印用法后退出 0；未知选项、缺值、超过 max_positional 个位置参数退出 1。
// 任何以 - 开头的未知参数都算参数错误，不会被当成路径（要传这种路径请写 ./xxx.xml）。
inline Args ParseArgs(int argc, char **argv, const char *usage, const std::vector<OptionDef> &defs,
                      size_t max_positional) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            std::printf("%s", usage);
            std::exit(0);
        }
        if (!a.empty() && a[0] == '-') {
            const OptionDef *def = nullptr;
            for (const OptionDef &d : defs) {
                if (a == d.name) {
                    def = &d;
                    break;
                }
            }
            if (def == nullptr) {
                std::fprintf(stderr, "未知参数：%s\n%s", a.c_str(), usage);
                std::exit(1);
            }
            std::string v;
            if (def->takes_value) {
                if (i + 1 >= argc) {
                    std::fprintf(stderr, "%s 后面缺参数\n%s", a.c_str(), usage);
                    std::exit(1);
                }
                v = argv[++i];
            }
            args.options[a] = v;
        } else if (args.positional.size() < max_positional) {
            args.positional.push_back(a);
        } else {
            std::fprintf(stderr, "位置参数最多 %zu 个：%s\n%s", max_positional, a.c_str(), usage);
            std::exit(1);
        }
    }
    return args;
}
