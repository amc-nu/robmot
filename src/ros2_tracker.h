#pragma once
#include <string>

// Blocks until shutdown. Needs parameters::selectedDetector to be set.
int runRos2(const std::string& inputTopic, const std::string& outputTopic);
