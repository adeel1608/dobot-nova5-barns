"""
oms_v1.gui

Streamlit-based production-point and marker-training GUI for the BARNS robot.

The package is intentionally a thin layer on top of:
    - oms_v1.manipulate_node.run_skill  (direct ROS2 path)
    - shared.rabbitmq_client.RabbitMQClient.execute_action  (fallback)
    - pickn_place tool_mount_teach / machine_mount_teach  (subprocess)
    - oms_v1.params  (AST-edited with backups)

No robot motion behavior is added or modified here. This package only reads
runtime state and updates static, teachable values on explicit operator
confirmation.

Entrypoint:
    streamlit run oms_v1/gui/streamlit_app.py
or, after `colcon build`:
    oms_v1_gui
"""
