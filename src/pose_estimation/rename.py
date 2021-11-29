import os
 
os.chdir('/home/user/workspace/src/pose_estimation/data/obj_1')
print("Exits. Please check path")
exit(0)

 
for count, f in enumerate(os.listdir()):
    f_name, f_ext = os.path.splitext(f)
    if f_ext == ".py":
        break
    f_name = "rgb_obj1_" + str(count)
    new_name = f'{f_name}{f_ext}'
    os.rename(f, new_name)
