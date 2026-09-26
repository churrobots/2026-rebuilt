# Core dashboard functionality

This directory contains the NetworkTables transport, connection selection, and simulated Driver Station. These are stable core functionality shared by simulation and the dashboard.

Before editing, moving, or deleting any file in this directory, stop and ask a coach for explicit approval. Do not make a speculative or incidental core change while working on custom-dashboard UI.

Keep the public interfaces narrow. Custom dashboard code may consume connection state and NetworkTables values, but must not publish `/SimSupervisor/*` topics or implement its own NetworkTables connection lifecycle.
