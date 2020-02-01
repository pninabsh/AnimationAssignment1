#pragma once
#include <string>
#include <iostream>
#include <windows.h>
#include <mmsystem.h>

class SoundManager
{
private:
	std::string catch_sound_path;
	std::string hit_sound_path;
	std::string end_level_sound_path;
public:
	void configureSoundPaths(std::string catch_sound_path, std::string hit_sound_path, std::string end_level_sound_path);
	void play_catch();
	void play_hit();
	void play_end_level();
};

