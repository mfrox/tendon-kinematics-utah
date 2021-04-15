#ifndef UTIL_OPENFILE_CHECK_H
#define UTIL_OPENFILE_CHECK_H

#include <string>

namespace util {

template <typename T>
void openfile_check(T& filestream, const std::string &filename) {
  // turn on exceptions on failure
  filestream.exceptions(std::ios::failbit);

  // opening will throw if the file does not exist or is not readable
  filestream.open(filename);

  // turn off all exceptions (back to the default behavior)
  filestream.exceptions(std::ios::goodbit);
}

} // end of namespace util

#endif // UTIL_OPENFILE_CHECK_H
