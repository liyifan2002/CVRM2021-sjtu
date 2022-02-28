#pragma once
#include "camera/general/video.h"

bool camera_grab_loop(bool &required_stop,bool &camera_is_ok);
bool video_file_loop(bool &required_stop, bool &camera_is_ok, const string& vid_path);
void debug_view_loop(bool &required_stop, bool &camera_is_ok);
void video_record_loop(const std::string &storage_location = "../data/");
