from vision_orchestrator_runner import run_orchestrator

result = run_orchestrator()
if result == 0:
    print("→ LEFT detected, turn left.")
elif result == 1:
    print("→ RIGHT detected, turn right.")
else:
    print("→ No detection.")