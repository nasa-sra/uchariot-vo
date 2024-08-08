#include <iostream>
#include <unistd.h>
#include <limits.h>
#include <string>

int main() {
  char currentDirectory[PATH_MAX];
  if (getcwd(currentDirectory, sizeof(currentDirectory)) == nullptr) {
    std::cerr << "Error getting current directory: " << errno << std::endl;
    return 1;
  }

  // Convert currentDirectory to a std::string
  std::string currentDirStr(currentDirectory);

  // Concatenate using std::string
  std::string voc_file = currentDirStr + "/Vocabulary/ORBvoc.txt"; 

  std::cout << "Vocabulary file path: " << voc_file << std::endl;
  return 0;
}