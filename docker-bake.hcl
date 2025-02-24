variable "TAG" {
  default = "latest"
}

group "default" {
  targets = ["builder"]
}

################################################################################
// MARK: General targets
################################################################################

target "baser" {
  target = "baser"
  tags = ["mtc:baser"]
  pull = false
  no-cache = false
}

target "cacher" {
  inherits   = ["baser"]
  target = "cacher"
  tags = ["mtc:cacher"]
}

target "runner" {
  inherits   = ["baser"]
  target = "runner"
  tags = ["mtc:runner"]
}

target "tester" {
  inherits   = ["runner"]
  target = "tester"
  tags = ["mtc:tester"]
}

target "builder" {
  inherits   = ["tester"]
  target = "builder"
  tags = ["mtc:builder"]
}

################################################################################
// MARK: Development targets 
################################################################################

variable "DEV_FROM_STAGE" {
  default = "builder"
}

target "dever" {
  inherits   = ["builder"]
  args = {
    DEV_FROM_STAGE = "${DEV_FROM_STAGE}",
  }
  target = "dever"
  tags = ["mtc:dever"]
}

target "seeder" {
  inherits   = ["dever"]
  target = "seeder"
  tags = ["mtc:seeder"]
  // no-cache-filter = ["compiler"]
  args = {
    CLEAR_WS_CACHE = null,
    // CLEAR_WS_CACHE = "${timestamp()}",
    SEED_WS_CACHE = null,
    // SEED_WS_CACHE = "${timestamp()}",
  }
}

target "compiler" {
  inherits   = ["seeder"]
  target = "compiler"
  tags = ["mtc:compiler"]
  // no-cache-filter = ["compiler"]
  args = {
    BUST_BUILD_CACHE = null,
    // BUST_BUILD_CACHE = "${timestamp()}",
  }
}

target "validator" {
  inherits   = ["compiler"]
  target = "validator"
  tags = ["mtc:validator"]
  args = {
    BUST_TEST_CACHE = null,
    // BUST_TEST_CACHE = "${timestamp()}",
  }
}

target "dancer" {
  inherits   = ["compiler"]
  target = "dancer"
  tags = ["mtc:dancer"]
}

target "exporter" {
  inherits   = ["dancer"]
  target = "exporter"
  tags = ["mtc:exporter"]
}

################################################################################
// MARK: Production targets
################################################################################

target "shipper" {
  inherits   = ["dancer"]
  target = "shipper"
}

variable "RUNNER" {
  default = "runner"
}

target "releaser" {
  inherits   = ["shipper"]
  target = "releaser"
  tags = ["mtc:releaser"]
  args = {
    SHIP_FROM_STAGE = "${RUNNER}",
  }
}

variable "BUILDER" {
  default = "builder"
}

target "debugger" {
  inherits   = ["shipper"]
  target = "debugger"
  tags = ["mtc:debugger"]
  args = {
    SHIP_FROM_STAGE = "${BUILDER}",
  }
}
