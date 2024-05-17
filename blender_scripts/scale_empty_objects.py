

def scale_selected_empty_objects(scale_factor):
    for obj in C.selected_objects:
        if obj.type == 'EMPTY':
            current_size = obj.empty_display_size 
            obj.empty_display_size = current_size * scale_factor

# Not used at the end. Scale operations where done on Blender gui by changing the transfrom pivot point to the 3D cursor located at the camera frame.
def scale_and_move_selected_objects(scale_factor):
    for obj in bpy.context.selected_objects:
        obj.dimensions = obj.dimensions * scale_factor
        obj.location = obj.location * scale_factor
        if obj.type == 'EMPTY':
            obj.empty_display_size = obj.empty_display_size * scale_factor

