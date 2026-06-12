# ============================================
# Robot Navigator — Makefile
# ============================================

.PHONY: help install run run-sim dashboard slam nav stop clean

PYTHON := python3
VENV := venv
PIP := $(VENV)/bin/pip
PY := $(VENV)/bin/python

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

# ── Setup ───────────────────────────────────────────────────

install: ## Install Python dependencies in virtual environment
	$(PYTHON) -m venv $(VENV)
	$(PIP) install --upgrade pip
	$(PIP) install -r requirements.txt
	@echo "\n✅ Installation complete! Activate with: source venv/bin/activate"

setup-env: ## Create .env file from template
	@if [ ! -f .env ]; then \
		cp .env.example .env; \
		echo "✅ Created .env — edit it with your API keys"; \
	else \
		echo "⚠️  .env already exists"; \
	fi

# ── Run ─────────────────────────────────────────────────────

run: ## Start the full robot system
	$(PY) src/main.py

run-sim: ## Start in simulation mode (no hardware required)
	PYTHONPATH=src $(PY) -m simulator.sim_runner

dashboard: ## Start only the web dashboard
	$(PY) -m uvicorn src.dashboard.server:app --host 0.0.0.0 --port 8080 --reload

# ── ROS 2 Nav2 (Ubuntu VM or Raspberry Pi) ──────────────────

run-nav2: ## Connect LLM agent to Nav2 via rosbridge (set ROSBRIDGE_HOST)
	PYTHONPATH=src $(PY) -m simulator.ros2_bridge

setup-vm: ## Show Ubuntu VM setup instructions
	@echo "Run this inside your Ubuntu VM:"
	@echo "  git clone https://github.com/Benzveen/robot-navigator.git ~/Development/robot"
	@echo "  cd ~/Development/robot"
	@echo "  chmod +x scripts/setup_ubuntu_vm.sh"
	@echo "  ./scripts/setup_ubuntu_vm.sh"

# ── ROS 2 (native, on Raspberry Pi) ────────────────────────

slam: ## Start SLAM mapping (ROS 2 on Pi)
	ros2 launch launch/robot.launch.py slam_mode:=mapping

nav: ## Start navigation with existing map (ROS 2 on Pi)
	ros2 launch launch/robot.launch.py slam_mode:=localization

save-map: ## Save the current SLAM map
	ros2 run nav2_map_server map_saver_cli -f maps/current_map

# ── Arduino ─────────────────────────────────────────────────

upload-arduino: ## Upload firmware to Arduino (requires arduino-cli)
	arduino-cli compile --fqbn arduino:avr:nano arduino/motor_controller/
	arduino-cli upload --fqbn arduino:avr:nano --port /dev/ttyUSB0 arduino/motor_controller/

monitor-arduino: ## Monitor Arduino serial output
	arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200

# ── Utilities ───────────────────────────────────────────────

stop: ## Emergency stop (sends stop command via serial)
	@echo '{"cmd":"stop"}' > /dev/ttyUSB0

logs: ## Tail the robot log file
	tail -f logs/robot.log

clean: ## Clean build artifacts and logs
	rm -rf __pycache__ src/__pycache__ src/**/__pycache__
	rm -rf *.egg-info build dist
	rm -rf logs/*.log

clean-all: clean ## Clean everything including venv and maps
	rm -rf $(VENV)
	rm -rf maps/*.pgm maps/*.yaml

# ── Development ─────────────────────────────────────────────

lint: ## Run linting
	$(PY) -m flake8 src/ --max-line-length 120 --ignore E501

format: ## Format code with black
	$(PY) -m black src/ --line-length 120

test: ## Run tests (no hardware or API key needed)
	$(PY) -m pytest tests/ -v

test-sim: ## Run simulation + integration tests
	$(PY) -m pytest tests/test_virtual_world.py tests/test_integration.py tests/test_agent.py -v

test-quick: ## Run fast unit tests only
	$(PY) -m pytest tests/test_virtual_world.py tests/test_agent.py -v --timeout=10
