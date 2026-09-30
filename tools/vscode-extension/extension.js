const vscode = require("vscode");

const SIM_TASK = "Start Supervised Robot Simulation";
const DASHBOARD_URL = "http://localhost:5800";
const CLAUDE_TERMINAL = "ChurroClaude";
// Keep in sync with the fill color in icons/*.svg.
const BUTTON_COLOR = "#FFC800";

function activate(context) {
  vscode.commands.executeCommand("setContext", "churrobots.robotProject", true);
  const buttons = [
    ["churrobots.startClaude", "$(sparkle) ChurroClaude", "Start a sandboxed Claude session (sbx run claude)", startClaude],
    ["churrobots.startSimulator", "$(play) ChurroSim", "Start the robot simulator (restarts on save)", startSimulator],
    ["churrobots.openDashboard", "$(dashboard) ChurroDashboard", `Open the robot dashboard (${DASHBOARD_URL})`, openDashboard],
  ];
  buttons.forEach(([command, text, tooltip, run], index) => {
    context.subscriptions.push(vscode.commands.registerCommand(command, run));
    const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100 - index);
    Object.assign(item, { command, text, tooltip, color: BUTTON_COLOR });
    item.show();
    context.subscriptions.push(item);
  });
}

function startClaude() {
  const existing = vscode.window.terminals.find((terminal) => terminal.name === CLAUDE_TERMINAL);
  if (existing) {
    existing.show();
    return;
  }
  const terminal = vscode.window.createTerminal({
    name: CLAUDE_TERMINAL,
    cwd: vscode.workspace.workspaceFolders?.[0]?.uri,
  });
  terminal.show();
  terminal.sendText("sbx run claude");
}

async function startSimulator() {
  if (vscode.tasks.taskExecutions.some((execution) => execution.task.name === SIM_TASK)) {
    vscode.window.showInformationMessage("The robot simulator is already running.");
    return;
  }
  const task = (await vscode.tasks.fetchTasks()).find((candidate) => candidate.name === SIM_TASK);
  if (!task) {
    vscode.window.showErrorMessage(`Couldn't find the "${SIM_TASK}" task in .vscode/tasks.json.`);
    return;
  }
  await vscode.tasks.executeTask(task);
}

function openDashboard() {
  vscode.env.openExternal(vscode.Uri.parse(DASHBOARD_URL));
}

module.exports = { activate, deactivate() {} };
