# 1. Go to the native project
cd ~/Documents/asymmetric-haptics/FalconBridge

# 2. Rebuild
cmake --build build --config Release

# 3. Copy the new plugin
cp -R build/FalconBridge.bundle \
  ~/Documents/asymmetric-haptics/Unity/FalconDemo/Assets/Plugins/macOS/

# 4. Restore the DHD library reference
install_name_tool \
  -change libdhd.3.17.7.dylib \
  @loader_path/../Frameworks/libdhd.3.17.7.dylib \
  ~/Documents/asymmetric-haptics/Unity/FalconDemo/Assets/Plugins/macOS/FalconBridge.bundle/Contents/MacOS/FalconBridge