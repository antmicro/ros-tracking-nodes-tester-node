namespace FileSystemUtils
{
	inline bool endsWith(std::string const & value, std::string const & ending);
	void listFiles(std::string dir_path, std::vector<std::string> &files, std::string extension="");
}
