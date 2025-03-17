from kaggle.api.kaggle_api_extended import KaggleApi

api = KaggleApi()
api.authenticate()

dataset_name = "hocop1/kitti-odometry"

# Get all image_3 files
files = [f.name for f in api.dataset_list_files(dataset_name).files if f.startswith("sequences/00/image_3/")]

print(f"Total images found in image_3: {len(files)}")
for img in files:
    print(img)
