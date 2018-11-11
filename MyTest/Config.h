#pragma once
#include <string>
#include <map>
#include <iostream>
using namespace std;
 
#define COMMENT_CHAR '#'
 
bool ReadConfig(const string & filename, map<string, string> & m);
void PrintConfig(const map<string, string> & m);