#!/bin/bash

sudo apt-get install git-lfs

# read password and put it in a variable
echo "Enter User for GITHUB"
read -s GITHUB_USER

echo "Enter password for GITHUB"
read -s GITHUB_PASSWORD
echo "user:" $GITHUB_USER
echo "password:" $GITHUB_PASSWORD

echo "---------------------------"

clone_repository() {

    echo "---------------------------"

    local name=$1
    local type=$2
    local url=$3
    local version=$4
    
    echo "Cloning repository: $name"
    
    # Clone the repository
    git clone --branch "$version" "$url" "$name"
    
    # Change to the repository directory
    pushd "$name" >/dev/null
    
    # Check if git-lfs is installed
    if command -v git-lfs >/dev/null 2>&1; then
        # Pull Git LFS objects
        git-lfs pull
    else
        echo "Warning: git-lfs not installed. Skipping git-lfs pull."
    fi
    
    # Change back to the original directory
    popd >/dev/null
}

# Call the function to clone repositories and run git-lfs pull
clone_repositories() {
    clone_repository "rrt_exploration" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/robosoft-ai/rrt_exploration.git" "main"
    clone_repository "SMACC2" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/robosoft-ai/SMACC2.git" "feature/humble_isaac_improvements"    
    clone_repository "isaac_ros_common" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git" "v2.0.0"
    clone_repository "isaac_ros_dnn_inference" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git" "v2.0.0"
    clone_repository "isaac_ros_image_pipeline" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git" "v2.0.0"
    clone_repository "isaac_ros_map_localization" "git" "https://github.com/pabloinigoblasco/isaac_ros_map_localization.git" "feature/tf_support_2.0.0"
    # clone_repository "isaac_ros_map_localization" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_map_localization.git" "v2.0.0"
    clone_repository "isaac_ros_nitros" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git" "v2.0.0"
    clone_repository "isaac_ros_object_detection" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git" "v2.0.0"
    clone_repository "isaac_ros_visual_slam" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git" "v2.0.0"
    clone_repository "isaac_ros_apriltag" "git" "https://$GITHUB_USER:$GITHUB_PASSWORD@github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git" "v2.0.0"
    clone_repository "pointcloud_to_laserscan" "git" "https://github.com/ros-perception/pointcloud_to_laserscan.git" "humble"

}

clone_repositories
