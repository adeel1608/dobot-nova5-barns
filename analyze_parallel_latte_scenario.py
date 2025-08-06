#!/usr/bin/env python3
"""
Analyze the optimal scheduling behavior for 2 lattes with 2 arms.
This shows how parallel processing should work while respecting dependencies.
"""

import json

def analyze_parallel_latte_scheduling():
    """Analyze optimal scheduling for 2 lattes with 2 arms."""
    
    print("🔍 PARALLEL LATTE SCHEDULING ANALYSIS")
    print("=" * 60)
    
    # Load recipes
    with open('/home/qss/BARNS/data/recipes.json', 'r') as f:
        recipes = json.load(f)
    
    # Simulate order with 2 lattes
    orders = [
        ("Latte", "order1-cup1"),
        ("Latte", "order1-cup2")
    ]
    
    print(f"📋 Analyzing order with {len(orders)} lattes:")
    for i, (drink, cup_id) in enumerate(orders, 1):
        print(f"  {i}. {drink} -> {cup_id}")
    print()
    
    # Create tasks
    tasks = []
    tasks_by_cup = {}
    completed = {}
    
    for drink, cup_id in orders:
        recipe = recipes[drink]
        completed[cup_id] = set()
        tasks_by_cup[cup_id] = []
        
        for step in recipe:
            action_name = step["action"]
            assigned_arm = step["assigned_arm"]
            dep_list = []
            if "depends_on" in step:
                deps = step["depends_on"]
                dep_list = deps if isinstance(deps, list) else [deps]
            task = {
                "drink": drink,
                "cup": cup_id,
                "action": action_name,
                "assigned_arm": assigned_arm,
                "depends_on": dep_list,
                "status": "pending"
            }
            tasks.append(task)
            tasks_by_cup[cup_id].append(task)
    
    print(f"📝 Total tasks created: {len(tasks)}")
    print()
    
    # Analyze task structure
    print("📊 LATTE RECIPE BREAKDOWN:")
    latte_recipe = recipes["Latte"]
    for step in latte_recipe:
        deps_str = ", ".join(step["depends_on"]) if step["depends_on"] else "None"
        print(f"  • {step['action']} ({step['assigned_arm']}) - Deps: {deps_str}")
    print()
    
    print("📊 TASK BREAKDOWN BY CUP:")
    for cup_id, cup_tasks in tasks_by_cup.items():
        print(f"\n🔸 {cup_id}:")
        for task in cup_tasks:
            deps_str = ", ".join(task["depends_on"]) if task["depends_on"] else "None"
            print(f"   • {task['action']} (Arm: {task['assigned_arm']}, Deps: {deps_str})")
    
    print("\n📊 TASKS BY ARM:")
    arm1_tasks = [t for t in tasks if t["assigned_arm"] == "Arm1"]
    arm2_tasks = [t for t in tasks if t["assigned_arm"] == "Arm2"]
    
    print(f"\n🤖 Arm1 Tasks ({len(arm1_tasks)}):")
    for task in arm1_tasks:
        deps_str = ", ".join(task["depends_on"]) if task["depends_on"] else "None"
        print(f"   • {task['cup']} - {task['action']} (Deps: {deps_str})")
    
    print(f"\n🤖 Arm2 Tasks ({len(arm2_tasks)}):")
    for task in arm2_tasks:
        deps_str = ", ".join(task["depends_on"]) if task["depends_on"] else "None"
        print(f"   • {task['cup']} - {task['action']} (Deps: {deps_str})")
    
    # Analyze dependencies
    print(f"\n🔗 DEPENDENCY ANALYSIS:")
    print(f"   • Arm1: Espresso → Espresso_pouring → Cleaning (sequential)")
    print(f"   • Arm2: Milk_Dispensing_Frothing (independent) → Milk_Pouring (needs Espresso from Arm1)")
    print(f"   • Cross-arm dependency: Milk_Pouring depends on Espresso")
    
    # Simulate optimal scheduling
    print(f"\n🎯 OPTIMAL PARALLEL SCHEDULING SIMULATION:")
    print("=" * 50)
    
    def get_available_tasks(arm_name, tasks, completed):
        """Get tasks available for an arm."""
        available = []
        for task in tasks:
            if task["assigned_arm"] == arm_name and task["status"] == "pending":
                cup_id = task["cup"]
                deps = task["depends_on"]
                if all(dep in completed[cup_id] for dep in deps):
                    available.append(task)
        return available
    
    def complete_task(task, completed):
        """Mark a task as completed."""
        task["status"] = "completed"
        cup_id = task["cup"]
        action = task["action"]
        completed[cup_id].add(action)
    
    # Simulate step-by-step execution
    step = 1
    time_step = 0
    
    print(f"{'Step':<4} {'Time':<4} {'Arm1':<25} {'Arm2':<25} {'Notes'}")
    print("-" * 80)
    
    # Initial state
    arm1_available = get_available_tasks("Arm1", tasks, completed)
    arm2_available = get_available_tasks("Arm2", tasks, completed)
    
    print(f"{step:<4} {time_step:<4} {'Starting...':<25} {'Starting...':<25} {'Initial state'}")
    step += 1
    time_step += 1
    
    # Step 1: Both arms can start independently
    # Arm1: Espresso (cup1), Arm2: Milk_Dispensing_Frothing (cup1)
    arm1_task = next((t for t in arm1_available if t["cup"] == "order1-cup1"), None)
    arm2_task = next((t for t in arm2_available if t["cup"] == "order1-cup1"), None)
    
    if arm1_task and arm2_task:
        complete_task(arm1_task, completed)
        complete_task(arm2_task, completed)
        arm1_desc = f"{arm1_task['action']} (cup1)"
        arm2_desc = f"{arm2_task['action']} (cup1)"
        print(f"{step:<4} {time_step:<4} {arm1_desc:<25} {arm2_desc:<25} {'Parallel execution'}")
        step += 1
        time_step += 1
    
    # Step 2: Arm1 continues cup1, Arm2 starts cup2
    arm1_available = get_available_tasks("Arm1", tasks, completed)
    arm2_available = get_available_tasks("Arm2", tasks, completed)
    
    arm1_task = next((t for t in arm1_available if t["cup"] == "order1-cup1"), None)
    arm2_task = next((t for t in arm2_available if t["cup"] == "order1-cup2"), None)
    
    if arm1_task and arm2_task:
        complete_task(arm1_task, completed)
        complete_task(arm2_task, completed)
        arm1_desc = f"{arm1_task['action']} (cup1)"
        arm2_desc = f"{arm2_task['action']} (cup2)"
        print(f"{step:<4} {time_step:<4} {arm1_desc:<25} {arm2_desc:<25} {'Arm2 starts cup2!'}")
        step += 1
        time_step += 1
    
    # Continue until all tasks complete or we hit a limit
    while step <= 10:
        arm1_available = get_available_tasks("Arm1", tasks, completed)
        arm2_available = get_available_tasks("Arm2", tasks, completed)
        
        if not arm1_available and not arm2_available:
            print(f"{step:<4} {time_step:<4} {'Idle':<25} {'Idle':<25} {'All tasks complete'}")
            break
        
        # Prioritize continuing current cup, then starting new cup
        arm1_task = None
        arm2_task = None
        
        # Arm1 selection
        if arm1_available:
            # First try to continue a cup already in progress
            for cup_id in ["order1-cup1", "order1-cup2"]:
                for task in arm1_available:
                    if task["cup"] == cup_id:
                        arm1_task = task
                        break
                if arm1_task:
                    break
        
        # Arm2 selection  
        if arm2_available:
            # First try to continue a cup already in progress
            for cup_id in ["order1-cup1", "order1-cup2"]:
                for task in arm2_available:
                    if task["cup"] == cup_id:
                        arm2_task = task
                        break
                if arm2_task:
                    break
        
        # Execute tasks
        notes = ""
        if arm1_task and arm2_task:
            complete_task(arm1_task, completed)
            complete_task(arm2_task, completed)
            if arm1_task["cup"] != arm2_task["cup"]:
                notes = "Different cups - parallel!"
            else:
                notes = "Same cup - coordinated"
        elif arm1_task:
            complete_task(arm1_task, completed)
            notes = "Arm2 waiting for dependencies"
        elif arm2_task:
            complete_task(arm2_task, completed)
            notes = "Arm1 waiting for dependencies"
        else:
            notes = "Both arms waiting"
        
        arm1_desc = f"{arm1_task['action']} ({arm1_task['cup'][-4:]})" if arm1_task else "Waiting"
        arm2_desc = f"{arm2_task['action']} ({arm2_task['cup'][-4:]})" if arm2_task else "Waiting"
        
        print(f"{step:<4} {time_step:<4} {arm1_desc:<25} {arm2_desc:<25} {notes}")
        
        step += 1
        time_step += 1
    
    # Show final completion status
    print(f"\n📊 FINAL COMPLETION STATUS:")
    for cup_id in ["order1-cup1", "order1-cup2"]:
        completed_tasks = len(completed[cup_id])
        total_tasks = len(tasks_by_cup[cup_id])
        status = "✅ Complete" if completed_tasks == total_tasks else f"⏳ {completed_tasks}/{total_tasks}"
        print(f"   {cup_id}: {status}")
        for action in completed[cup_id]:
            print(f"     ✓ {action}")
    
    return True

if __name__ == "__main__":
    analyze_parallel_latte_scheduling()
    
    print(f"\n" + "=" * 60)
    print(f"🎯 KEY INSIGHTS:")
    print(f"✅ Arms should work on different cups when possible")
    print(f"✅ Each arm should complete its tasks for a cup before switching")
    print(f"✅ Cross-arm dependencies (Milk_Pouring needs Espresso) must be respected")
    print(f"✅ Optimal: Arm finishes cup1 → can start cup2 while other arm works")
    print(f"=" * 60) 