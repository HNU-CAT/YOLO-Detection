#ifndef UTILS_H
#define UTILS_H

#include <dirent.h>
#include <sys/stat.h>
#include <random>
#include <opencv2/opencv.hpp>

static inline int read_files_in_dir(const char *p_dir_name, std::vector<std::string> &file_names)
{
    DIR *p_dir = opendir(p_dir_name);
    if (p_dir == nullptr)
    {
        return -1;
    }

    struct dirent *p_file = nullptr;
    while ((p_file = readdir(p_dir)) != nullptr)
    {
        if (strcmp(p_file->d_name, ".") != 0 &&
            strcmp(p_file->d_name, "..") != 0)
        {
            // std::string cur_file_name(p_dir_name);
            // cur_file_name += "/";
            // cur_file_name += p_file->d_name;
            std::string cur_file_name(p_file->d_name);
            file_names.push_back(cur_file_name);
        }
    }

    closedir(p_dir);
    return 0;
}

static inline int get_random_int(int minThres = 0, int maxThres = 255)
{
    // 获取处于某一范围内的一个随机整数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(minThres, maxThres);

    int random_integer = distrib(gen);

    return random_integer;
}

static inline int read_files_in_dir(const std::string &image_path, std::vector<std::string> &file_names)
{
    DIR *p_dir = opendir(image_path.c_str());
    if (!p_dir)
        return -1;

    struct dirent *p_file;
    while ((p_file = readdir(p_dir)) != nullptr)
    {
        if (strcmp(p_file->d_name, ".") == 0 || strcmp(p_file->d_name, "..") == 0)
        {
            continue;
        }

        std::string full_path = image_path + "/" + p_file->d_name;

        // 检查是否为普通文件
        struct stat file_stat;
        if (stat(full_path.c_str(), &file_stat) != 0 || !S_ISREG(file_stat.st_mode))
        {
            continue;
        }

        const char *ext = strrchr(p_file->d_name, '.');
        if (!ext)
            continue; // 无扩展名则跳过
        std::string extension(ext);
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

        const std::vector<std::string> allowed_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".gif"};
        if (std::find(allowed_extensions.begin(), allowed_extensions.end(), extension) != allowed_extensions.end())
        {
            file_names.push_back(p_file->d_name); // 仅存储文件名
        }
    }
    closedir(p_dir);
    return 0;
}

#endif // UTILS_H
