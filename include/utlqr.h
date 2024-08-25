#ifndef UTILITYBR_H
#define UTILITYBR_H

#include <string>

namespace utlqr // ytility qt-rviz
{
  static std::string get_root_path()
  {
    char *p = NULL;

    constexpr int len = 256;
    /// to keep the absolute path of executable's path
    char arr_tmp[len] = {0};

    int n = readlink("/proc/self/exe", arr_tmp, len);
    if (n > len)
    {
      LOG(WARNING) << "exe path is too long!";
    }
    if (NULL != (p = strrchr(arr_tmp, '/')))
    {
      *p = '\0';
    }
    else
    {
      return std::string("");
    }

    std::string path = std::string(std::filesystem::path(std::string(arr_tmp)).parent_path());

    return path;
  }
}

#endif