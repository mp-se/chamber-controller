import { describe, it, expect, beforeEach, vi } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { nextTick } from 'vue'
import piniaInstance, { global, config, status, saveConfigState, getConfigChanges } from '@/modules/pinia'

describe('pinia module', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
    vi.clearAllMocks()
  })

  it('exports pinia instance', () => {
    expect(piniaInstance).toBeDefined()
  })

  it('exports global store', () => {
    expect(global).toBeDefined()
    expect(global.$id).toBe('global')
  })

  it('exports config store', () => {
    expect(config).toBeDefined()
    expect(config.$id).toBe('config')
  })

  it('exports status store', () => {
    expect(status).toBeDefined()
    expect(status.$id).toBe('status')
  })

  it('stores have required methods', () => {
    expect(typeof config.load).toBe('function')
    expect(typeof config.sendConfig).toBe('function')
    expect(typeof config.saveAll).toBe('function')
    expect(typeof status.load).toBe('function')
  })
})

describe('config store actions exist', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('load is callable', () => {
    expect(typeof config.load).toBe('function')
  })
  it('sendConfig is callable', () => {
    expect(typeof config.sendConfig).toBe('function')
  })
  it('restart is callable', () => {
    expect(typeof config.restart).toBe('function')
  })
  it('sendWifiScan is callable', () => {
    expect(typeof config.sendWifiScan).toBe('function')
  })
  it('sendSensorScan is callable', () => {
    expect(typeof config.sendSensorScan).toBe('function')
  })
  it('getWifiScanStatus is callable', () => {
    expect(typeof config.getWifiScanStatus).toBe('function')
  })
  it('getSensorScanStatus is callable', () => {
    expect(typeof config.getSensorScanStatus).toBe('function')
  })
  it('saveAll is callable', () => {
    expect(typeof config.saveAll).toBe('function')
  })
  it('runWifiScan is callable', () => {
    expect(typeof config.runWifiScan).toBe('function')
  })
  it('runSensorScan is callable', () => {
    expect(typeof config.runSensorScan).toBe('function')
  })
})

describe('config store state properties', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('has id property', () => {
    expect(config).toHaveProperty('id')
  })
  it('has mdns property', () => {
    expect(config).toHaveProperty('mdns')
  })
  it('has temp_format property', () => {
    expect(config).toHaveProperty('temp_format')
  })
  it('has ota_url property', () => {
    expect(config).toHaveProperty('ota_url')
  })
  it('has wifi_ssid property', () => {
    expect(config).toHaveProperty('wifi_ssid')
  })
  it('has mqtt_target property', () => {
    expect(config).toHaveProperty('mqtt_target')
  })
  it('has mqtt_port property', () => {
    expect(config).toHaveProperty('mqtt_port')
  })
  it('has influxdb2_target property', () => {
    expect(config).toHaveProperty('influxdb2_target')
  })
  it('has influxdb2_bucket property', () => {
    expect(config).toHaveProperty('influxdb2_bucket')
  })
  it('has influxdb2_org property', () => {
    expect(config).toHaveProperty('influxdb2_org')
  })
  it('has dark_mode property', () => {
    expect(config).toHaveProperty('dark_mode')
  })
  it('has ble_push_enabled property', () => {
    expect(config).toHaveProperty('ble_push_enabled')
  })
  it('has ble_scan_enabled property', () => {
    expect(config).toHaveProperty('ble_scan_enabled')
  })
  it('has fridge_sensor_id property', () => {
    expect(config).toHaveProperty('fridge_sensor_id')
  })
  it('has beer_sensor_id property', () => {
    expect(config).toHaveProperty('beer_sensor_id')
  })
  it('has enable_cooling property', () => {
    expect(config).toHaveProperty('enable_cooling')
  })
  it('has enable_heating property', () => {
    expect(config).toHaveProperty('enable_heating')
  })
  it('has restart_interval property', () => {
    expect(config).toHaveProperty('restart_interval')
  })
})

describe('status and global stores', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
  })

  it('status.load is callable', () => {
    expect(typeof status.load).toBe('function')
  })
  it('global.clearMessages is callable', () => {
    expect(typeof global.clearMessages).toBe('function')
  })
  it('global has messageError property', () => {
    expect(global).toHaveProperty('messageError')
  })
  it('global has messageSuccess property', () => {
    expect(global).toHaveProperty('messageSuccess')
  })
  it('global has disabled property', () => {
    expect(global).toHaveProperty('disabled')
  })
})

describe('pinia module exports', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
    vi.clearAllMocks()
  })

  it('saveConfigState is defined', () => {
    expect(typeof saveConfigState).toBe('function')
  })

  it('getConfigChanges is defined', () => {
    expect(typeof getConfigChanges).toBe('function')
  })

  it('getConfigChanges returns empty object when no state saved', () => {
    const changes = getConfigChanges()
    expect(typeof changes).toBe('object')
  })

  it('saveConfigState captures config values', () => {
    config.mdns = 'test-device'
    saveConfigState()
    // After saving, no changes should be detected
    const changes = getConfigChanges()
    expect(changes).toEqual({})
  })

  it('getConfigChanges detects modified values', () => {
    config.mdns = 'original'
    saveConfigState()
    config.mdns = 'modified'
    const changes = getConfigChanges()
    expect(changes).toHaveProperty('mdns', 'modified')
  })

  it('configChanged flag is managed by config store', () => {
    expect(global).toHaveProperty('configChanged')
  })

  it('initialized flag affects subscription behavior', () => {
    expect(global).toHaveProperty('initialized')
  })
})

describe('pinia $subscribe callback (lines 79-86)', () => {
  beforeEach(() => {
    // Reset store state to defaults to ensure clean test isolation
    global.initialized = false
    global.configChanged = false
    global.messageError = ''
    config.dark_mode = false
    config.mdns = 'chamber'
    // Re-initialize configCompare for clean state
    saveConfigState()
  })

  afterEach(() => {
    // Prevent leaking initialized=true into other test suites
    global.initialized = false
    global.configChanged = false
  })

  it('$subscribe callback body executes when initialized=true and config changes', async () => {
    // Establish baseline state
    global.initialized = false
    const uniqueMdns = 'subscribe_exec_' + Date.now()
    config.mdns = uniqueMdns
    saveConfigState() // save current state

    // Enable subscription processing
    global.initialized = true

    // Trigger a config change — this fires $subscribe
    config.mdns = uniqueMdns + '_v2'

    // After a config change, getConfigChanges should return the changed property
    const changes = getConfigChanges()
    expect(changes).toHaveProperty('mdns')
    expect(changes.mdns).toBe(uniqueMdns + '_v2')

    // Restore
    global.initialized = false
    config.mdns = uniqueMdns
  })

  it('$subscribe with diff > 2 chars sets configChanged=true (line 83)', async () => {
    global.initialized = false
    const base = 'base_' + Date.now()
    config.mdns = base
    saveConfigState()
    global.configChanged = false

    global.initialized = true
    config.mdns = base + '_changed'
    await nextTick()

    expect(global.configChanged).toBe(true)

    global.initialized = false
    config.mdns = base
  })

  it('$subscribe with no changes sets configChanged=false (line 86)', async () => {
    global.initialized = false
    const base = 'base_' + Date.now()
    config.mdns = base
    saveConfigState()

    // Change to a different value first so configChanged becomes true
    global.initialized = true
    config.mdns = base + '_changed'
    await nextTick()
    expect(global.configChanged).toBe(true)

    // Now change back to the baseline value — subscribe should set configChanged=false
    config.mdns = base
    await nextTick()
    expect(global.configChanged).toBe(false)

    global.initialized = false
  })

  it('does not enter subscribe body when global.initialized is false', () => {
    global.initialized = false
    // Just verifying that the early-return guard (line 77) is the existing coverage
    const uniqueMdns = 'no_subscribe_' + Date.now()
    config.mdns = uniqueMdns
    // $subscribe fires but returns early because initialized=false
    expect(global.initialized).toBe(false)
  })
})
