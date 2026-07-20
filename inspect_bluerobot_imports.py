with open(r'C:/Users/tungh/Downloads/Quickstart-master/Quickstart-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/BLUEROBOT.java', 'r', encoding='utf-8-sig') as f:
    text = f.read()

lines = text.splitlines()
print("Imports in BLUEROBOT.java:")
for i in range(0, min(40, len(lines))):
    print(f"{i+1}: {lines[i]}")
