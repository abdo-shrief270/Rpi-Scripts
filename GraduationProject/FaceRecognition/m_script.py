import CamModule as camera_module


captured_photo_path=camera_module.capture_image()
print(captured_photo_path)

# 5 is the duration in Sec
#recorder_video_path=camera_module.record_video(10)
#print(recorder_video_path)
