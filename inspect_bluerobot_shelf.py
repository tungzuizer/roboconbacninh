with open(r'C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/BLUEROBOT.java', 'r', encoding='utf-8-sig') as f:
    text = f.read()

lines = text.splitlines()

# Tìm shelfRoutine và in chi tiết 20 dòng trước và 25 dòng sau
for i, line in enumerate(lines):
    if 'Command shelfRoutine(' in line:
        print(f"Found shelfRoutine at line {i+1}")
        for j in range(max(0, i-5), min(i+25, len(lines))):
            print(f"{j+1}: {lines[j]}")
        break
