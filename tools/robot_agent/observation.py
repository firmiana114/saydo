"""Explicit camera provenance; pixel coordinates never transfer between views."""
def label_frame(frame, source_tool):
    frame=dict(frame)
    role='movable_global' if source_tool=='observe_global' else 'eye_in_hand'
    frame.update(camera_role=role,source_tool=source_tool,
                 camera_label='不固定的全局相机' if role=='movable_global' else '眼在手机位')
    if role=='eye_in_hand':
        frame['camera_serial']=(frame.get('camera') or {}).get('serial')
        frame['usage_note']='眼在手画面，随机械臂移动；框选仅用于本机位，不是独立全局观察。'
    return frame
