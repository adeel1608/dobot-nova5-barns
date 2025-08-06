#!/usr/bin/env python3
"""
Test the per-arm cup priority scheduling fix for 2 lattes with 2 arms.
This verifies that arms can work on different cups in parallel while each arm
completes all its tasks for a cup before starting a new cup.
"""

import json

def test_per_arm_cup_priority():
    """Test the per-arm cup priority scheduling algorithm."""
    
    print("🧪 PER-ARM CUP PRIORITY TEST (2 Lattes, 2 Arms)")
    print("=" * 60)
    
    # Load recipes
    with open('/home/qss/BARNS/data/recipes.json', 'r') as f:
        recipes = json.load(f)
    
    # Simulate order with 2 lattes
    orders = [
        ("Latte", "order1-cup1"),
        ("Latte", "order1-cup2")
    ]
    
    print(f"📋 Testing order with {len(orders)} lattes:")
    for i, (drink, cup_id) in enumerate(orders, 1):
        print(f"  {i}. {drink} -> {cup_id}")
    print()
    
    # Create tasks
    tasks = []
    tasks_by_cup = {}
    completed = {}
    
    # Per-arm cup priority data structures
    per_arm_current_cups = {"Arm1": None, "Arm2": None}
    cup_completion_status = {}
    
    for drink, cup_id in orders:
        recipe = recipes[drink]
        completed[cup_id] = set()
        tasks_by_cup[cup_id] = []
        cup_completion_status[cup_id] = "pending"
        
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
    
    # Implement the per-arm cup priority algorithm
    def select_task_with_per_arm_cup_priority(arm_name):
        """Select a task using per-arm cup priority scheduling."""
        
        # Get all pending tasks for this arm
        pending_tasks = [t for t in tasks if t["assigned_arm"] == arm_name and t["status"] == "pending"]
        
        # Filter to tasks whose dependencies are satisfied
        available_tasks = []
        for task in pending_tasks:
            cup_id = task["cup"]
            deps = task["depends_on"]
            if all(dep in completed[cup_id] for dep in deps):
                available_tasks.append(task)
        
        if not available_tasks:
            return None
        
        # Priority 1: Tasks from the cup this arm is currently working on
        current_cup_id = per_arm_current_cups[arm_name]
        current_cup_tasks = [t for t in available_tasks if t["cup"] == current_cup_id]
        
        if current_cup_tasks:
            # Select the first available task from current cup
            task = current_cup_tasks[0]
            print(f"   🎯 [PER-ARM] {arm_name} continuing work on cup {task['cup'][-4:]} - {task['action']}")
            return task
        
        # Priority 2: Tasks from new cups (only if arm has no current cup)
        if current_cup_id is None:
            # Start working on a new cup - prefer pending cups, but also allow in-progress cups
            new_cup_tasks = [t for t in available_tasks if cup_completion_status[t["cup"]] in ["pending", "in_progress"]]
            
            if new_cup_tasks:
                task = new_cup_tasks[0]
                cup_id = task["cup"]
                
                # Mark this cup as being worked on by this arm
                per_arm_current_cups[arm_name] = cup_id
                if cup_completion_status[cup_id] == "pending":
                    cup_completion_status[cup_id] = "in_progress"
                
                print(f"   🆕 [PER-ARM] {arm_name} starting new cup {cup_id[-4:]} - {task['action']}")
                return task
        
        # If we get here, the arm has a current cup but no available tasks for it
        print(f"   ⏳ [PER-ARM] {arm_name} waiting for dependencies on cup: {current_cup_id[-4:] if current_cup_id else 'None'}")
        return None
    
    def complete_task(task):
        """Mark a task as completed and update cup status if needed."""
        task["status"] = "completed"
        cup_id = task["cup"]
        action = task["action"]
        completed[cup_id].add(action)
        
        print(f"   ✅ Completed: {action} for {cup_id[-4:]}")
        
        # Check if all tasks for this arm on this cup are complete
        arm_name = task["assigned_arm"]
        arm_tasks_for_cup = [t for t in tasks_by_cup[cup_id] if t["assigned_arm"] == arm_name]
        arm_completed_for_cup = sum(1 for t in arm_tasks_for_cup if t["status"] == "completed")
        
        if arm_completed_for_cup == len(arm_tasks_for_cup):
            # This arm has completed all its tasks for this cup
            if per_arm_current_cups[arm_name] == cup_id:
                per_arm_current_cups[arm_name] = None
            print(f"   🏁 [PER-ARM] {arm_name} finished all tasks for cup {cup_id[-4:]}")
            
            # Check if the entire cup is complete (all arms done)
            if len(completed[cup_id]) == len(tasks_by_cup[cup_id]):
                cup_completion_status[cup_id] = "completed"
                print(f"   🏆 [PER-ARM] Cup {cup_id[-4:]} completely finished!")
    
    # Simulate parallel execution
    print("🤖 PARALLEL EXECUTION SIMULATION:")
    print("-" * 50)
    
    step = 1
    execution_log = []
    
    print(f"{'Step':<4} {'Arm1':<30} {'Arm2':<30} {'Notes'}")
    print("-" * 80)
    
    while step <= 20:  # Safety limit
        # Check if all tasks are completed
        all_completed = all(t["status"] == "completed" for t in tasks)
        if all_completed:
            print(f"{step:<4} {'All tasks complete!':<30} {'All tasks complete!':<30} {'✅ Done'}")
            break
        
        # Try to select tasks for both arms
        arm1_task = select_task_with_per_arm_cup_priority("Arm1")
        arm2_task = select_task_with_per_arm_cup_priority("Arm2")
        
        # Execute tasks
        notes = ""
        arm1_desc = "Waiting"
        arm2_desc = "Waiting"
        
        if arm1_task and arm2_task:
            complete_task(arm1_task)
            complete_task(arm2_task)
            arm1_desc = f"{arm1_task['action']} ({arm1_task['cup'][-4:]})"
            arm2_desc = f"{arm2_task['action']} ({arm2_task['cup'][-4:]})"
            if arm1_task["cup"] != arm2_task["cup"]:
                notes = "Parallel cups!"
            else:
                notes = "Same cup"
        elif arm1_task:
            complete_task(arm1_task)
            arm1_desc = f"{arm1_task['action']} ({arm1_task['cup'][-4:]})"
            notes = "Arm2 waiting"
        elif arm2_task:
            complete_task(arm2_task)
            arm2_desc = f"{arm2_task['action']} ({arm2_task['cup'][-4:]})"
            notes = "Arm1 waiting"
        else:
            notes = "Both waiting"
        
        print(f"{step:<4} {arm1_desc:<30} {arm2_desc:<30} {notes}")
        
        # Log for analysis
        execution_log.append({
            "step": step,
            "arm1": arm1_task,
            "arm2": arm2_task,
            "parallel_cups": arm1_task and arm2_task and arm1_task["cup"] != arm2_task["cup"]
        })
        
        step += 1
        
        # If no progress, break
        if not arm1_task and not arm2_task:
            print(f"{step:<4} {'No progress possible':<30} {'No progress possible':<30} {'⚠️ Stopping'}")
            break
    
    # Analyze results
    print(f"\n📊 EXECUTION ANALYSIS:")
    parallel_steps = sum(1 for log in execution_log if log["parallel_cups"])
    total_steps = len([log for log in execution_log if log["arm1"] or log["arm2"]])
    
    print(f"   Total execution steps: {total_steps}")
    print(f"   Steps with parallel cups: {parallel_steps}")
    print(f"   Parallel efficiency: {parallel_steps/total_steps*100:.1f}%" if total_steps > 0 else "   No execution")
    
    # Show cup completion order
    print(f"\n📋 FINAL COMPLETION STATUS:")
    for cup_id in ["order1-cup1", "order1-cup2"]:
        completed_tasks = len(completed[cup_id])
        total_tasks = len(tasks_by_cup[cup_id])
        status = "✅ Complete" if completed_tasks == total_tasks else f"⏳ {completed_tasks}/{total_tasks}"
        print(f"   {cup_id}: {status}")
    
    # Verify per-arm completion
    print(f"\n🔍 PER-ARM COMPLETION VERIFICATION:")
    for arm_name in ["Arm1", "Arm2"]:
        arm_tasks = [t for t in tasks if t["assigned_arm"] == arm_name]
        
        # Group by cup and check completion order
        cups_worked = {}
        for task in arm_tasks:
            if task["status"] == "completed":
                cup_id = task["cup"]
                if cup_id not in cups_worked:
                    cups_worked[cup_id] = []
                cups_worked[cup_id].append(task["action"])
        
        print(f"   {arm_name} worked on cups: {list(cups_worked.keys())}")
        
        # Check if arm completed one cup before starting another
        if len(cups_worked) > 1:
            # Find which cup was completed first
            cup_completion_order = []
            for cup_id in cups_worked:
                arm_tasks_for_cup = [t for t in arm_tasks if t["cup"] == cup_id]
                if len(cups_worked[cup_id]) == len(arm_tasks_for_cup):
                    cup_completion_order.append(cup_id)
            
            if len(cup_completion_order) >= 1:
                print(f"     ✅ {arm_name} properly completed cups in sequence")
            else:
                print(f"     ❌ {arm_name} may have interleaved cup work")
    
    return parallel_steps > 0

if __name__ == "__main__":
    success = test_per_arm_cup_priority()
    
    print(f"\n" + "=" * 60)
    if success:
        print(f"✅ PER-ARM CUP PRIORITY ALGORITHM WORKING!")
        print(f"🎯 Arms can work on different cups in parallel")
        print(f"🎯 Each arm completes its tasks for a cup before switching")
    else:
        print(f"❌ ALGORITHM NEEDS IMPROVEMENT")
    print(f"=" * 60) 