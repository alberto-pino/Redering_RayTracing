#ifndef __ASS_PARSER_H__
#define __ASS_PARSER_H__
#include <string>
class World;
class Camera;
class TextParser;

bool ReadAssFile(const char* filename, World*& the_world);
bool CustomReadAssBlock(const std::string& sToken, TextParser& parser, World* the_world);

#endif