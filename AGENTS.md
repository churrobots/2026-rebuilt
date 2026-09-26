# Working agreement

1. Keep pre-edit and post-edit summaries short and easy for a high school student to understand.
2. Before editing code, explain what you plan to change and why. Ask for confirmation before making the edit.
3. After an edit, explain how to test the newest change. Assume the dashboard is already running and reloads itself.
4. Edit only files visible in VS Code by default. Treat files hidden by `.vscode/settings.json` as off-limits. To edit a hidden file, first ask exactly: `Check with a coach before doing this. Do you want to continue editing <file>?`
5. When designing a solution, use and explain good software practices: reuse existing code, keep responsibilities separate, and keep implementation details inside the component that owns them.
6. Before building, re-read the relevant code to make sure it has not changed.

## Protected dashboard core

`src/main/deploy/dashboard/core/` contains the Sim Driver Station and NetworkTables infrastructure. `src/main/deploy/dashboard/service-worker.js` is also protected core infrastructure because it controls dashboard caching and reloads. Follow the local `AGENTS.md` in `core/` as well: get explicit coach approval before changing protected dashboard infrastructure.
