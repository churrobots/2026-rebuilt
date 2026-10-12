const vscode = require("vscode");

const SIM_TASK = "Start Supervised Robot Simulation";
const DUMP_TASK = "Download Robot Debug Info";
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
    ["churrobots.dumpDebug", "$(cloud-download) ChurroDump", "Save recent robot logs and NetworkTables values to debug/", dumpDebug],
  ];
  buttons.forEach(([command, text, tooltip, run], index) => {
    context.subscriptions.push(vscode.commands.registerCommand(command, run));
    const item = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 100 - index);
    Object.assign(item, { command, text, tooltip, color: BUTTON_COLOR });
    item.show();
    context.subscriptions.push(item);
  });
}

/** @type {vscode.Terminal | undefined} */
let claudeTerminal;

function startClaude() {
  // Only reuse a terminal we started this session. Terminals VS Code restores after a restart
  // are plain shells with the same name, and Claude isn't running in them.
  if (claudeTerminal && claudeTerminal.exitStatus === undefined && vscode.window.terminals.includes(claudeTerminal)) {
    claudeTerminal.show();
    return;
  }
  claudeTerminal = vscode.window.createTerminal({
    name: CLAUDE_TERMINAL,
    cwd: vscode.workspace.workspaceFolders?.[0]?.uri,
    isTransient: true,
  });
  claudeTerminal.show();
  claudeTerminal.sendText("sbx run claude");
}

function startSimulator() {
  return runTask(SIM_TASK, "The robot simulator is already running.");
}

function dumpDebug() {
  return runTask(DUMP_TASK, "Already downloading robot debug info.");
}

async function runTask(name, alreadyRunningMessage) {
  if (vscode.tasks.taskExecutions.some((execution) => execution.task.name === name)) {
    vscode.window.showInformationMessage(alreadyRunningMessage);
    return;
  }
  const task = (await vscode.tasks.fetchTasks()).find((candidate) => candidate.name === name);
  if (!task) {
    vscode.window.showErrorMessage(`Couldn't find the "${name}" task in .vscode/tasks.json.`);
    return;
  }
  await vscode.tasks.executeTask(task);
}

function openDashboard() {
  vscode.env.openExternal(vscode.Uri.parse(DASHBOARD_URL));
}

module.exports = { activate, deactivate() {} };
