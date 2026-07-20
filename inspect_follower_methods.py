import os

# Tìm file Follower.java trong project
target = "Follower.java"
found_path = ""
for root, dirs, files in os.walk(r'C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master'):
    if target in files:
        found_path = os.path.join(root, target)
        break

if found_path:
    with open(found_path, 'r', encoding='utf-8') as f:
        content = f.read()
    print(f"Found Follower.java at {found_path}")
    # Tìm các phương thức chứa "set" hoặc "Drive"
    lines = content.splitlines()
    for line in lines:
        if "public" in line and "(" in line and ")" in line:
             if "set" in line or "Drive" in line or "pose" in line.lower():
                 print(line.strip())
else:
    print("Follower.java not found")
