import os
import shutil

def move_files_to_folders(root_folder):
    # Define source and destination folders
    src_folder = os.path.join(root_folder, "src")
    include_folder = os.path.join(root_folder, "include")

    # Create destination folders if they don't exist
    if not os.path.exists(src_folder):
        os.makedirs(src_folder)
    if not os.path.exists(include_folder):
        os.makedirs(include_folder)

    # Recursively traverse all directories within the root folder
    for root, dirs, files in os.walk(root_folder):
        for filename in files:
            file_path = os.path.join(root, filename)
            if filename.endswith(".c"):
                # Move C files to src folder
                shutil.move(file_path, os.path.join(src_folder, filename))
            elif filename.endswith(".h"):
                # Move header files to include folder
                shutil.move(file_path, os.path.join(include_folder, filename))

if __name__ == "__main__":
    folder_path = "/Users/elakhyanedumaran/Documents/Tinympc/Final/teensy_rl_benchmark/lib/ecos/"
    print('path',folder_path)
    move_files_to_folders(folder_path)
    print("Files moved successfully.")

