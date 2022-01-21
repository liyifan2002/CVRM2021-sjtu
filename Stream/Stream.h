#pragma once
#include "video.h"

bool camera_grab_loop(bool &required_stop,bool &camera_is_ok);
bool video_file_loop(bool &required_stop, bool &camera_is_ok, const string& vid_path);
void show_debug_view(bool &required_stop,bool &camera_is_ok);
void video_record(const std::string &storage_location = "../data/");
