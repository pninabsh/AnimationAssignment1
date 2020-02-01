#include "SoundManager.h"


void SoundManager::configureSoundPaths( std::string catch_sound_path, std::string hit_sound_path, std::string end_level_sound_path) {
	this->catch_sound_path = catch_sound_path;
	this->hit_sound_path = hit_sound_path;
	this->end_level_sound_path = end_level_sound_path;
}

void SoundManager::play_catch() {
	PlaySound(this->catch_sound_path.c_str(), NULL, SND_ASYNC);

}

void SoundManager::play_hit() {
	PlaySound(this->hit_sound_path.c_str(), NULL, SND_ASYNC | SND_NOWAIT);
}

void SoundManager::play_end_level() {
	PlaySound(this->end_level_sound_path.c_str(), NULL, SND_ASYNC);
}