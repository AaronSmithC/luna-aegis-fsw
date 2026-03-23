# ══════════════════════════════════════════════════════════════
# Luna-Aegis Short Hopper — Flight Software Build
# ══════════════════════════════════════════════════════════════
#
# Targets:
#   make all       — Build all apps + HAL (sim mode)
#   make check     — Compile check only (no link)
#   make clean     — Remove build artifacts
#   make console   — Serve ground console on localhost:8080
#
# Design Authority: Aegis Station Infrastructure LLC
# ══════════════════════════════════════════════════════════════

CC      = gcc
CFLAGS  = -Wall -Wextra -Wpedantic -std=c11 -g -O2
CFLAGS += -I fsw/cfe_hdr -I fsw/msg_defs -I fsw/hal/inc
CFLAGS += -DSIM_MODE

# ── Source Discovery ──
HAL_SIM_SRC  = fsw/hal/sim/la_hal_sim.c
APP_SRCS     = $(wildcard fsw/apps/*/src/*.c)

# ── Build directory ──
BUILD_DIR = build

# ── Object files ──
HAL_OBJ  = $(BUILD_DIR)/la_hal_sim.o
APP_OBJS = $(patsubst fsw/apps/%/src/%.c,$(BUILD_DIR)/%.o,$(APP_SRCS))

.PHONY: all check clean console tree

# ── Compile check (no link — we don't have a real cFE to link against) ──
all: check
	@echo ""
	@echo "══════════════════════════════════════════════════"
	@echo "  Luna-Aegis FSW — Compile check PASSED"
	@echo "  Apps compiled: $$(ls $(BUILD_DIR)/*.o 2>/dev/null | wc -l) objects"
	@echo "══════════════════════════════════════════════════"

check: $(BUILD_DIR)
	@echo "Compiling HAL (sim mode)..."
	$(CC) $(CFLAGS) -c $(HAL_SIM_SRC) -o $(BUILD_DIR)/la_hal_sim.o
	@echo "Compiling mission apps..."
	@for src in $(APP_SRCS); do \
		obj=$$(basename $$src .c).o; \
		echo "  CC $$src"; \
		$(CC) $(CFLAGS) -c $$src -o $(BUILD_DIR)/$$obj || exit 1; \
	done
	@echo "All sources compiled successfully."

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

console:
	@echo "Serving ground console on http://localhost:8080"
	@cd ground/console && python3 -m http.server 8080

tree:
	@find . -type f -not -path './build/*' -not -path './.git/*' \
		| sort | head -80
