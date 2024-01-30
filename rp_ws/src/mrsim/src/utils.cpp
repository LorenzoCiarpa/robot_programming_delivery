#include "utils.h"

void killTerminal() {
# if defined(_WIN32) || defined(_WIN64)
    system("pause");
    system("TASKKILL /F /IM cmd.exe");
  #elif defined(__unix__) || defined(__APPLE__)
    cout << "Closing the terminal" << endl;
    sleep(1);
    system("kill -9 $(ps -p $PPID -o ppid=)");
  #else
    #error "Unknown system"
  #endif
}

void clearTerminal() {
  // Move the cursor to the start position for the messages
  std::cout << "\033[" << 12 << ";1H";
  // Clear the line
  std::cout << "\033[K";
}


int runShellCommand(string command) {
  int result = system(command.c_str());
  if (result != 0) {
    ROS_ERROR("Failed to execute shell command");
    return 1;
  }
  return 0;
}

Json::Value readJson( string in_path) {
  
  cout << "Configuration file path:" << in_path <<  endl;
  Json::Value root; 
  Json::CharReaderBuilder readerBuilder; 

   ifstream file(in_path,  ifstream::binary); 
   cout <<in_path <<  endl;
   string errs; 

  if (!file.is_open()) {
    cerr << "Could not open the file: " << in_path <<  endl;
    return Json::Value();
  }
  
  bool parsingSuccessful = Json::parseFromStream(readerBuilder, file, &root, &errs);
  if (!parsingSuccessful) {
      cout << "Failed to parse JSON file: " << errs <<  endl;
      return Json::Value();
  }

  file.close();
  return root;
}
