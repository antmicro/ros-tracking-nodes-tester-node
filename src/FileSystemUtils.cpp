#include <FileSystemUtils.hpp>
#include <dirent.h>
#include <cstring>

inline bool FileSystemUtils::endsWith(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void FileSystemUtils::listFiles(std::string dir_path, std::vector<std::string> &files, std::string extension)
{
    DIR *dir;
    struct dirent *entry;

    if (!(dir = opendir(dir_path.c_str())))
        return;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_DIR) {
            char path[1024];
            if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
                continue;
            snprintf(path, sizeof(path), "%s/%s", dir_path.c_str(), entry->d_name);
            listFiles(path, files, extension);
        } else {
            std::string file(entry->d_name);
            if (endsWith(file, extension)) {
                files.push_back(dir_path+"/"+entry->d_name);
            }
        }
    }
    closedir(dir);
}
