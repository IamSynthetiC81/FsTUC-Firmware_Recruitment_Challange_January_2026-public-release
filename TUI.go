package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"runtime"
	"strings"
	"time"

	"github.com/charmbracelet/bubbles/spinner"
	"github.com/charmbracelet/bubbles/viewport"
	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/lipgloss"
)

// --- Constants & Types ---
type focus int
const (
	focusSidebar focus = iota
	focusViewport
)

type tickMsg time.Time // Used for automatic file polling

// --- Styling ---
var (
	headerStyle = lipgloss.NewStyle().
			Foreground(lipgloss.Color("#FFF7DB")).
			Background(lipgloss.Color("#5A56E0")).
			Padding(0, 1).Bold(true).MarginBottom(1)

	activeBorder   = lipgloss.Color("#5A56E0")
	inactiveBorder = lipgloss.Color("#3C3C3C")

	sidebarStyle = lipgloss.NewStyle().
			Border(lipgloss.NormalBorder(), false, true, false, false).
			Padding(0, 1).Width(35)

	logBoxStyle = lipgloss.NewStyle().
			Border(lipgloss.RoundedBorder()).
			Padding(0, 1)

	warnStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("#FFA500")).Italic(true)
	passStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("#00FF00")).Bold(true)
	failStyle = lipgloss.NewStyle().Foreground(lipgloss.Color("#FF0000")).Bold(true)
)

type Task struct {
	Label      string
	Steps      [][]string
	Cwd        string // optional working directory; empty means workspace root
	ShowOutput bool   // when true, include captured output even on success
}

type model struct {
	focus         focus
	cursor        int
	tasks         []Task
	running       bool
	viewport      viewport.Model
	spinner       spinner.Model
	width, height int
	ready         bool
	platform      string
	lastBuild     time.Time
	dirtyFiles    []string
}

// cmakeGenerator returns the best CMake -G value for the current host.
// Ninja is preferred when available; falls back to platform-native generators.
func cmakeGenerator() string {
	if _, err := exec.LookPath("ninja"); err == nil {
		return "Ninja"
	}
	switch runtime.GOOS {
	case "windows":
		if _, err := exec.LookPath("mingw32-make"); err == nil {
			return "MinGW Makefiles"
		}
		if _, err := exec.LookPath("mingw64-make"); err == nil {
			return "MinGW Makefiles"
		}
		return "NMake Makefiles"
	default: // linux, darwin
		return "Unix Makefiles"
	}
}

// exeSuffix returns ".exe" on Windows, empty string elsewhere.
func exeSuffix() string {
	if runtime.GOOS == "windows" {
		return ".exe"
	}
	return ""
}

func initialModel() model {
	s := spinner.New(spinner.WithSpinner(spinner.Dot), spinner.WithStyle(lipgloss.NewStyle().Foreground(lipgloss.Color("#5A56E0"))))
	
	m := model{
		platform: fmt.Sprintf("%s (%s)", runtime.GOOS, runtime.GOARCH),
		spinner:  s,
		focus:    focusSidebar,
		tasks: []Task{
			{Label: "Setup CMake",          Steps: [][]string{{"cmake", "-B", "build", "-G", cmakeGenerator()}}},
			{Label: "Build",                Steps: [][]string{{"cmake", "--build", "build"}}},
			{Label: "Rebuild (clean+build)", Steps: [][]string{
				{"cmake", "--build", "build", "--target", "clean"},
				{"cmake", "-B", "build", "-G", cmakeGenerator()},
				{"cmake", "--build", "build"},
			}},
			{Label: "Clean",               Steps: [][]string{{"cmake", "--build", "build", "--target", "clean"}}},
			// {Label: "Run Contactor Tests",  Steps: [][]string{{filepath.Join("build", "test_contactor"+exeSuffix())}},  ShowOutput: true},
			// {Label: "Run BatteryPack Tests", Steps: [][]string{{filepath.Join("build", "test_battery_pack"+exeSuffix())}}, ShowOutput: true},
			{Label: "Run All Tests",        Steps: [][]string{{"ctest", "--verbose", "--parallel", "4"}}, Cwd: "build"},
		},
	}
	m.refreshBuildStatus()
	return m
}

// --- Logic: Updated for Clean Reflection and Watch List ---
func (m *model) refreshBuildStatus() {
	buildDir := "build"
	// Files that indicate a valid build exists
	artifacts := []string{"CMakeCache.txt", "test_contactor", "test_battery_pack"}
	// Files to watch for changes
	watched := []string{
		filepath.Join("lib", "src", "contactor.c"),
		filepath.Join("lib", "src", "BatteryPack.c"),
		filepath.Join("lib", "src", "FSM.c"),
	}

	var latest time.Time
	foundAny := false
	for _, a := range artifacts {
		if info, err := os.Stat(filepath.Join(buildDir, a)); err == nil {
			foundAny = true
			if info.ModTime().After(latest) {
				latest = info.ModTime()
			}
		}
	}

	// If no artifacts found (e.g. after Clean), reset time to zero
	if !foundAny {
		m.lastBuild = time.Time{}
	} else {
		m.lastBuild = latest
	}

	m.dirtyFiles = []string{}
	for _, w := range watched {
		if info, err := os.Stat(w); err == nil {
			// If never built, or source is newer than last build artifact
			if m.lastBuild.IsZero() || info.ModTime().After(m.lastBuild) {
				m.dirtyFiles = append(m.dirtyFiles, filepath.Base(w))
			}
		}
	}
}

// Automatic interval command
func doTick() tea.Cmd {
	return tea.Tick(time.Second*2, func(t time.Time) tea.Msg {
		return tickMsg(t)
	})
}

func (m model) Init() tea.Cmd {
	return tea.Batch(m.spinner.Tick, doTick())
}

func (m model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	var cmd tea.Cmd

	switch msg := msg.(type) {
	case tickMsg:
		m.refreshBuildStatus()
		return m, doTick() // Schedule next tick

	case tea.KeyMsg:
		switch msg.String() {
		case "q":
			return m, tea.Quit
		case "ctrl+c":
			m.viewport.SetContent("Viewport cleared.")
			return m, nil
		case "left":
			m.focus = focusSidebar
		case "right":
			m.focus = focusViewport
		case "up", "down", "k", "j":
			if m.focus == focusSidebar {
				if msg.String() == "up" || msg.String() == "k" {
					if m.cursor > 0 { m.cursor-- }
				} else {
					if m.cursor < len(m.tasks)-1 { m.cursor++ }
				}
			} else {
				m.viewport, cmd = m.viewport.Update(msg)
				return m, cmd
			}
		case "enter":
			if m.focus == focusSidebar && !m.running {
				m.running = true
				return m, m.runTask(m.tasks[m.cursor])
			}
		}

	case tea.WindowSizeMsg:
		m.width, m.height = msg.Width, msg.Height
		if !m.ready {
			m.viewport = viewport.New(msg.Width-40, msg.Height-10)
			m.ready = true
		} else {
			m.viewport.Width, m.viewport.Height = msg.Width-40, msg.Height-10
		}

	case logMsg:
		m.running = false
		m.refreshBuildStatus() // Immediate refresh after task finish
		content := m.viewport.View() + "\n" + string(msg)
		m.viewport.SetContent(content)
		m.viewport.GotoBottom()

	case spinner.TickMsg:
		m.spinner, cmd = m.spinner.Update(msg)
		return m, cmd
	}

	return m, nil
}

func (m model) View() string {
	if !m.ready { return "Starting..." }

	var sb strings.Builder
	sb.WriteString(lipgloss.NewStyle().Bold(true).Render("ACTIONS") + "\n\n")
	for i, t := range m.tasks {
		prefix := "  "
		style := lipgloss.NewStyle().Foreground(inactiveBorder)
		if m.cursor == i {
			prefix, style = "> ", lipgloss.NewStyle().Foreground(activeBorder).Bold(true)
		}
		sb.WriteString(style.Render(prefix+t.Label) + "\n")
	}

	sb.WriteString("\n" + lipgloss.NewStyle().Underline(true).Render("BUILD STATUS") + "\n")
	if m.lastBuild.IsZero() {
		sb.WriteString("Last Build: " + lipgloss.NewStyle().Foreground(lipgloss.Color("#888888")).Render("None/Cleaned") + "\n")
	} else {
		sb.WriteString(fmt.Sprintf("Last Build: %s\n", m.lastBuild.Format("15:04:05")))
	}

	if len(m.dirtyFiles) > 0 {
		sb.WriteString("\n" + warnStyle.Render("⚠ Out of Date:") + "\n")
		for _, f := range m.dirtyFiles {
			sb.WriteString(warnStyle.Render(" • "+f) + "\n")
		}
	} else if !m.lastBuild.IsZero() {
		sb.WriteString("\n" + lipgloss.NewStyle().Foreground(lipgloss.Color("#00FF00")).Render("✓ Build Current") + "\n")
	}

	sideColor, logColor := inactiveBorder, inactiveBorder
	if m.focus == focusSidebar { sideColor = activeBorder } else { logColor = activeBorder }

	sidebar := sidebarStyle.BorderForeground(sideColor).Height(m.height - 8).Render(sb.String())
	logs := logBoxStyle.BorderForeground(logColor).Render(m.viewport.View())

	header := headerStyle.Render("FsTUC Build Tool | " + m.platform)
	help := "\n (q) quit | (ctrl+c) clear | (←/→) focus | Auto-refresh: ON"
	if m.running { help = "\n " + m.spinner.View() + " Running Task..." }

	return lipgloss.JoinVertical(lipgloss.Left, header, lipgloss.JoinHorizontal(lipgloss.Top, sidebar, logs), help)
}

type logMsg string

// relativizeOutput replaces every occurrence of the absolute workspace root
// (plus path separator) in raw command output with a leading ".", keeping all
// displayed paths relative and significantly shorter.
func relativizeOutput(raw []byte, workspaceAbs string) string {
	s := string(raw)
	// Replace both forward-slash and OS-native separator variants so the
	// substitution works on Windows (backslash) and Linux/macOS (slash) alike.
	slash := string(filepath.Separator)
	s = strings.ReplaceAll(s, workspaceAbs+slash, "."+slash)
	if slash != "/" {
		// Also handle forward-slash variants embedded in CMake/ctest output
		// on Windows (CMake often emits "/" even on Windows).
		s = strings.ReplaceAll(s, filepath.ToSlash(workspaceAbs)+"/", "./")
	}
	return s
}

// Compiled once at startup; case-insensitive word-boundary patterns for
// Unity/CTest result tokens. Word boundaries prevent FAIL matching inside
// FAILED (and vice-versa) regardless of replacement order.
var (
	reColorFailed = regexp.MustCompile(`(?i)\bFAILED\b`)
	reColorPassed = regexp.MustCompile(`(?i)\bPASSED\b`)
	reColorFail   = regexp.MustCompile(`(?i)\bFAIL\b`)
	reColorPass   = regexp.MustCompile(`(?i)\bPASS\b`)
)

// colorizeOutput replaces Unity/CTest result words with ANSI-coloured
// equivalents. Matching is case-insensitive; word boundaries ensure that
// FAIL does not fire inside FAILED and PASS does not fire inside PASSED.
func colorizeOutput(s string) string {
	s = reColorFailed.ReplaceAllStringFunc(s, func(m string) string { return failStyle.Render(m) })
	s = reColorPassed.ReplaceAllStringFunc(s, func(m string) string { return passStyle.Render(m) })
	s = reColorFail.ReplaceAllStringFunc(s, func(m string) string { return failStyle.Render(m) })
	s = reColorPass.ReplaceAllStringFunc(s, func(m string) string { return passStyle.Render(m) })
	return s
}

func (m model) runTask(t Task) tea.Cmd {
	return func() tea.Msg {
		var out strings.Builder
		timestamp := time.Now().Format("15:04:05")

		// Resolve workspace root once; used for path relativization in output.
		workspaceAbs, err := filepath.Abs(".")
		if err != nil {
			workspaceAbs = ""
		}

		// Resolve working directory: default to process cwd (workspace root).
		stepDir := t.Cwd
		if stepDir != "" {
			if abs, err := filepath.Abs(stepDir); err == nil {
				stepDir = abs
			}
		}

		for _, step := range t.Steps {
			cmd := exec.Command(step[0], step[1:]...)
			if stepDir != "" {
				cmd.Dir = stepDir
			}
			output, _ := cmd.CombinedOutput()
			out.WriteString(colorizeOutput(relativizeOutput(output, workspaceAbs)))
			if cmd.ProcessState != nil && !cmd.ProcessState.Success() {
				return logMsg(fmt.Sprintf("[%s] %s %s\n%s", timestamp, failStyle.Render("FAILED:"), t.Label, out.String()))
			}
		}
		if t.ShowOutput {
			return logMsg(fmt.Sprintf("[%s] %s %s\n%s", timestamp, passStyle.Render("PASSED:"), t.Label, out.String()))
		}
		return logMsg(fmt.Sprintf("[%s] %s %s", timestamp, passStyle.Render("PASSED:"), t.Label))
	}
}

func main() {
	p := tea.NewProgram(initialModel(), tea.WithAltScreen())
	if _, err := p.Run(); err != nil { os.Exit(1) }
}