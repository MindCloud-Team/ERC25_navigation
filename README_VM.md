# Mind Cloud – ERC25 Bag Playback & Web UI Guide

## 1. Accessing the VM Webtop

We use **Webtop** to interact with the VM environment remotely.
It allows you to run GUI applications inside the VM without installing anything locally.

**Link:**

```
https://[fc94:4f67:5cb2:cc7d:530d:47b1:af70:e919]:3001/
```

**Credentials:**

* **Username:** `mindcloud`
* **Password:** `mind1234`
---

## 2. Downloading and Extracting a Bag File

We store and play back ROS 2 bag files for testing and development.

```bash
# Download the file
curl -L -o erc25.zip "<link>"

# -L: follow redirects
# -o: specify output file name

# Extract the zip
unzip -o erc25.zip
```

---

## 3. Checking and Playing a Bag File

Once extracted, you can inspect and replay the bag:

```bash
# Show bag file info
ros2 bag info <bag_filename>

# Play bag file in a loop
ros2 bag play <bag_filename> --loop
```

**Example (our current dataset):**

```bash
cd ~/mind_cloud/bag
ros2 bag play rosbag2_2025_08_05-13_17_06/ --loop
```

---

## 4. Checking if a Bag File is Already Running

Before running a bag file, **make sure no one else is playing it**:

```bash
ps aux | grep bag
```

**Example output when the bag is running:**

```
team     2482237  7.4  0.8 1086084 103860 pts/1  Sl+  07:45   0:00 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag play rosbag2_2025_08_05-13_17_06/ --loop
team     2482298  0.0  0.0  11676  2288 pts/2    S+   07:45   0:00 grep --color=auto bag
```

If you see **more than 2 lines**, it means another playback is active.

---

⚠ **Note:**
* The bag file has a video attached with it in the same directory.
* The recorded video file is **6GB** — do **NOT** download it to your PC.
* You can watch it directly from Webtop.

## 5. Running the Web UI

The **Web UI** provides an interface to view sensor data and camera feeds without depending on local ROS setups.
It only requires a working **Husarnet VPN** connection.

### Launch the Web UI:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch web_ui web_ui.launch.py topics_config:=/home/team/ros2_ws/src/ERC25_navigation/web_ui/web_ui/www/config/topics_real.json
```

* **`topics_real.json`** – used for real robot data.
* **`topics.json`** – used for simulation data.
  Both are located in:

  ```
  ~/ros2_ws/src/ERC25_navigation/web_ui/web_ui/www/config/
  ```

If a topic name changes, edit the correct JSON file accordingly.

---

## 6. Accessing the Web UI in Webtop

Once the Web UI is running, open **Firefox inside Webtop** and visit:

```
localhost:8080
```

---

## 7. Camera Decompression

We have scripts to decompress camera feeds into raw image topics.

From your **home directory (`~`)**:

```bash
# Start decompression
./start_camera.sh

# Stop decompression
./stop_camera.sh
```

---

✅ **Summary Workflow:**

1. Log in to Webtop.
2. Check if a bag file is already running.
3. If free, play the bag in a loop.
4. Start camera decompression (if needed).
5. Launch the Web UI.
6. Open `localhost:8080` in Webtop’s Firefox to view the interface.
