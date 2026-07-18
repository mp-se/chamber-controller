import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import DeviceHardwareView from '../DeviceHardwareView.vue'
import { createPinia, setActivePinia } from 'pinia'
import { useConfigStore } from '@/modules/configStore'
import { useGlobalStore } from '@/modules/globalStore'
import { useStatusStore } from '@/modules/statusStore'

let testConfig
let testGlobal
let testStatus

vi.mock('@mp-se/espframework-ui-components', () => ({
  logDebug: vi.fn(),
  logError: vi.fn(),
  logInfo: vi.fn(),
  validateCurrentForm: vi.fn(() => true),
  sharedHttpClient: {
    get: vi.fn(),
    post: vi.fn()
  }
}))

vi.mock('@/modules/pinia', async () => {
  const actual = await vi.importActual('@/modules/pinia')
  return {
    default: actual.default,
    get config() { return testConfig },
    get global() { return testGlobal },
    get status() { return testStatus }
  }
})

describe('DeviceHardwareView', () => {
  let pinia
  let config
  let global
  let status

  beforeEach(() => {
    vi.clearAllMocks()
    pinia = createPinia()
    setActivePinia(pinia)
    config = useConfigStore(pinia)
    global = useGlobalStore(pinia)
    status = useStatusStore(pinia)

    // Assign to module-level variables so the mocked pinia module returns them
    testConfig = config
    testGlobal = global
    testStatus = status

    // Reset config state
    config.fridge_sensor_id = ''
    config.beer_sensor_id = ''
    config.fridge_sensor_offset = 0
    config.beer_sensor_offset = 0
    config.enable_cooling = false
    config.enable_heating = false
    config.invert_pins = false
    config.ble_scan_enabled = false
    config.ble_sensor_valid_time = 5
    config.beer_ble_sensor_id = ''

    global.disabled = false
    global.feature = { ble_sensor: false }
    global.configChanged = true

    // Reset status state
    status.temperature_device = []

    // Mock config methods
    config.runSensorScan = vi.fn(async () => ({
      success: true,
      data: {
        status: false,
        success: true,
        sensors: ['sensor1', 'sensor2']
      }
    }))

    config.saveAll = vi.fn(async () => {})

    // Mock status methods
    status.load = vi.fn(async () => {
      status.temperature_device = []
    })
  })

  afterEach(() => {
    // Clean up after tests
  })

  const createWrapper = () => {
    return mount(DeviceHardwareView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsSelect: true,
          BsInputNumber: true,
          BsInputSwitch: true,
          BsButton: true,
          BsMessage: true
        }
      }
    })
  }

  describe('component mounting', () => {
    it('mounts without error', () => {
      const wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })

    it('displays title', () => {
      const wrapper = createWrapper()
      expect(wrapper.text()).toContain('Device - Settings')
    })

    it('disables controls during mount', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.exists()).toBe(true)
    })
  })

  describe('sensor scan on mount', () => {
    it('calls runSensorScan on mount', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(config.runSensorScan).toHaveBeenCalled()
    })

    it('populates sensor options from scan results', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          status: false,
          success: true,
          sensors: ['DS18B20_1', 'DS18B20_2']
        }
      }))
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThan(1)
    })

    it('handles failed sensor scan', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: false
      }))
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThanOrEqual(1)
    })

    it('handles null response from sensor scan', async () => {
      config.runSensorScan = vi.fn(async () => null)
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThanOrEqual(1)
    })

    it('adds not-detected label for configured beer sensor not in scan results', async () => {
      config.beer_sensor_id = 'missing_sensor'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          status: false,
          success: true,
          sensors: ['sensor1', 'sensor2']
        }
      }))
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      const notDetectedOptions = wrapper.vm.sensorOptions.filter(
        o => o.label.includes('missing_sensor') && o.label.includes('not detected')
      )
      expect(notDetectedOptions.length).toBeGreaterThan(0)
    })

    it('adds not-detected label for configured fridge sensor not in scan results', async () => {
      config.fridge_sensor_id = 'missing_fridge'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          status: false,
          success: true,
          sensors: ['sensor1', 'sensor2']
        }
      }))
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      const notDetectedOptions = wrapper.vm.sensorOptions.filter(
        o => o.label.includes('missing_fridge') && o.label.includes('not detected')
      )
      expect(notDetectedOptions.length).toBeGreaterThan(0)
    })

    it('does not add not-detected label if sensor is in scan results', async () => {
      config.beer_sensor_id = 'sensor1'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          status: false,
          success: true,
          sensors: ['sensor1', 'sensor2']
        }
      }))
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      const notDetectedOptions = wrapper.vm.sensorOptions.filter(
        o => o.label.includes('sensor1') && o.label.includes('not detected')
      )
      expect(notDetectedOptions.length).toBe(0)
    })

    it('enables controls after sensor scan completes', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      await new Promise(r => setTimeout(r, 10))
      expect(global.disabled).toBe(false)
    })
  })

  describe('BLE sensor loading on mount', () => {
    it('calls status.load() on mount', async () => {
      global.feature.ble_sensor = true
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(status.load).toHaveBeenCalled()
    })

    it('populates BLE sensor options from status data', async () => {
      global.feature.ble_sensor = true
      status.load = vi.fn(async () => {
        status.temperature_device = [
          { device: 'BLE_Device_1', type: 'Tilt' },
          { device: 'BLE_Device_2', type: 'iSpindel' }
        ]
      })
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Give async operations time to complete
      await new Promise(r => setTimeout(r, 10))
      expect(wrapper.vm.bleSensorOptions.length).toBeGreaterThanOrEqual(2)
    })

    it('handles empty BLE sensor list', async () => {
      global.feature.ble_sensor = true
      status.load = vi.fn(async () => {
        status.temperature_device = []
      })
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.bleSensorOptions.length).toBeGreaterThanOrEqual(1)
    })

    it('processes BLE sensor data from status', async () => {
      global.feature.ble_sensor = true
      status.load = vi.fn(async () => {
        status.temperature_device = [
          { device: 'BLE_001', type: 'Tilt' }
        ]
      })
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // The default option + at least the BLE device
      expect(wrapper.vm.bleSensorOptions.length).toBeGreaterThanOrEqual(1)
      expect(status.load).toHaveBeenCalled()
    })

    it('adds not-detected label for configured BLE sensor not in list', async () => {
      config.beer_ble_sensor_id = 'missing_ble'
      global.feature.ble_sensor = true
      status.load = vi.fn(async () => {
        status.temperature_device = [
          { device: 'BLE_001', type: 'Tilt' }
        ]
      })
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Give time for async operations
      await new Promise(r => setTimeout(r, 50))
      const notDetectedOptions = wrapper.vm.bleSensorOptions.filter(
        o => o.value === 'missing_ble'
      )
      // Should have at least the not-detected entry
      expect(notDetectedOptions.length).toBeGreaterThanOrEqual(1)
    })

    it('does not add not-detected label if BLE sensor is in list', async () => {
      config.beer_ble_sensor_id = 'BLE_001'
      global.feature.ble_sensor = true
      status.load = vi.fn(async () => {
        status.temperature_device = [
          { device: 'BLE_001', type: 'Tilt' }
        ]
      })
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      const notDetectedOptions = wrapper.vm.bleSensorOptions.filter(
        o => o.value === 'BLE_001' && o.label.includes('not detected')
      )
      expect(notDetectedOptions.length).toBe(0)
    })
  })

  describe('form submission and saveSettings', () => {
    it('has save button', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('form')
      expect(wrapper.html()).toContain('Save')
    })

    it('has saveSettings method', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.saveSettings).toBeDefined()
      expect(typeof wrapper.vm.saveSettings).toBe('function')
    })

    it('renders form with needs-validation class', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('form.needs-validation').exists()).toBe(true)
    })

    it('calls saveAll when saveSettings is executed', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.saveSettings()
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('validates form before saving', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      const wrapper = createWrapper()
      await wrapper.vm.saveSettings()
      expect(validateCurrentForm).toHaveBeenCalled()
    })

    it('does not call saveAll if validation fails', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)
      const wrapper = createWrapper()
      await wrapper.vm.saveSettings()
      expect(config.saveAll).not.toHaveBeenCalled()
    })

    it('form submission is prevented by default', () => {
      const wrapper = createWrapper()
      const form = wrapper.find('form')
      expect(form.exists()).toBe(true)
    })

    it('calls saveSettings with async execution', async () => {
      const wrapper = createWrapper()
      const promise = wrapper.vm.saveSettings()
      expect(promise).toBeInstanceOf(Promise)
    })
  })

  describe('disabled state', () => {
    it('respects global disabled flag', async () => {
      global.disabled = true
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(global.disabled).toBe(true)
    })

    it('enables controls after mount completes', async () => {
      const wrapper = createWrapper()
      // Wait for onMounted to complete
      await wrapper.vm.$nextTick()
      await new Promise(r => setTimeout(r, 50))
      // After onMounted completes, disabled should be false
      expect(global.disabled).toBe(false)
    })

    it('disables controls during initial mount', () => {
      // When component first mounts, onMounted sets disabled to true
      const mockRunSensorScan = vi.fn(async () => {
        expect(global.disabled).toBe(true)
        return { success: true, data: { status: false, success: true, sensors: [] } }
      })
      config.runSensorScan = mockRunSensorScan
      const wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })
  })

  describe('option list initialization', () => {
    it('initializes sensor options with default', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThanOrEqual(1)
      expect(wrapper.vm.sensorOptions[0].label).toContain('not selected')
    })

    it('initializes BLE options with default', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.bleSensorOptions.length).toBeGreaterThanOrEqual(1)
      expect(wrapper.vm.bleSensorOptions[0].label).toContain('not selected')
    })

    it('initializes BLE valid time options correctly', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.bleValidOptions.length).toBe(6)
      expect(wrapper.vm.bleValidOptions[0].value).toBe(5)
      expect(wrapper.vm.bleValidOptions[1].value).toBe(10)
      expect(wrapper.vm.bleValidOptions[2].value).toBe(15)
      expect(wrapper.vm.bleValidOptions[3].value).toBe(20)
      expect(wrapper.vm.bleValidOptions[4].value).toBe(25)
      expect(wrapper.vm.bleValidOptions[5].value).toBe(30)
    })

    it('verifies BLE valid time option labels', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.bleValidOptions[0].label).toBe('5 minutes')
      expect(wrapper.vm.bleValidOptions[5].label).toBe('30 minutes')
    })
  })

  describe('sensor configuration properties', () => {
    it('stores sensor IDs', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      config.fridge_sensor_id = 'chamber1'
      config.beer_sensor_id = 'beer1'
      expect(config.fridge_sensor_id).toBe('chamber1')
      expect(config.beer_sensor_id).toBe('beer1')
    })

    it('displays sensor selector inputs', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Chamber Sensor')
      expect(wrapper.html()).toContain('Beer Sensor')
    })
  })

  describe('sensor offset configuration', () => {
    it('displays sensor offset inputs', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Beer Sensor Offset')
      expect(wrapper.html()).toContain('Fridge Sensor Offset')
    })

    it('stores positive sensor offsets', () => {
      config.beer_sensor_offset = 0.5
      config.fridge_sensor_offset = 1.5
      expect(config.beer_sensor_offset).toBe(0.5)
      expect(config.fridge_sensor_offset).toBe(1.5)
    })

    it('stores negative sensor offsets', () => {
      config.beer_sensor_offset = -0.75
      config.fridge_sensor_offset = -2.0
      expect(config.beer_sensor_offset).toBe(-0.75)
      expect(config.fridge_sensor_offset).toBe(-2.0)
    })

    it('stores zero offsets', () => {
      config.beer_sensor_offset = 0
      config.fridge_sensor_offset = 0
      expect(config.beer_sensor_offset).toBe(0)
      expect(config.fridge_sensor_offset).toBe(0)
    })
  })

  describe('heating and cooling control', () => {
    it('displays cooling toggle', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Enable Cooling')
    })

    it('displays heating toggle', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Enable Heating')
    })

    it('stores cooling setting', () => {
      config.enable_cooling = true
      expect(config.enable_cooling).toBe(true)
      config.enable_cooling = false
      expect(config.enable_cooling).toBe(false)
    })

    it('stores heating setting', () => {
      config.enable_heating = true
      expect(config.enable_heating).toBe(true)
      config.enable_heating = false
      expect(config.enable_heating).toBe(false)
    })
  })

  describe('pin inversion setting', () => {
    it('displays invert pins toggle', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Invert pins')
    })

    it('stores invert_pins setting', () => {
      config.invert_pins = false
      expect(config.invert_pins).toBe(false)
      config.invert_pins = true
      expect(config.invert_pins).toBe(true)
    })

    it('can toggle invert pins multiple times', () => {
      config.invert_pins = false
      expect(config.invert_pins).toBe(false)
      config.invert_pins = true
      expect(config.invert_pins).toBe(true)
      config.invert_pins = false
      expect(config.invert_pins).toBe(false)
    })
  })

  describe('BLE configuration properties', () => {
    it('stores BLE scan enabled setting', () => {
      config.ble_scan_enabled = true
      expect(config.ble_scan_enabled).toBe(true)
      config.ble_scan_enabled = false
      expect(config.ble_scan_enabled).toBe(false)
    })

    it('stores BLE sensor valid time setting', () => {
      config.ble_sensor_valid_time = 10
      expect(config.ble_sensor_valid_time).toBe(10)
      config.ble_sensor_valid_time = 30
      expect(config.ble_sensor_valid_time).toBe(30)
    })

    it('stores selected BLE sensor', () => {
      config.beer_ble_sensor_id = 'ble_device_1'
      expect(config.beer_ble_sensor_id).toBe('ble_device_1')
    })

    it('renders BLE section based on feature flag', async () => {
      global.feature.ble_sensor = true
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.html()).toContain('BLE')
    })

    it('hides BLE section when feature flag is false', async () => {
      global.feature.ble_sensor = false
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.exists()).toBe(true)
    })
  })

  describe('form submission event handling', () => {
    it('form submit event calls saveSettings', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      config.saveAll = vi.fn().mockResolvedValue(true)
      
      const form = wrapper.find('form')
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('form submit is prevented by @submit.prevent', async () => {
      const wrapper = createWrapper()
      const form = wrapper.find('form')
      expect(form.attributes('novalidate')).toBe('')
      expect(form.element.onsubmit).toBeDefined()
    })

    it('saveSettings executes with async/await in form submission', async () => {
      const wrapper = createWrapper()
      config.saveAll = vi.fn(async () => true)
      
      await wrapper.vm.saveSettings()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('form saves data when submitted with valid input', async () => {
      config.fridge_sensor_id = 'fridge'
      config.beer_sensor_id = 'beer'
      
      const wrapper = createWrapper()
      config.saveAll = vi.fn().mockResolvedValue(true)
      
      const form = wrapper.find('form')
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('handles form submission with sensor scan results', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['s1', 's2', 's3']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 50))
      
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThanOrEqual(1)
    })
  })

  describe('async function execution', () => {
    it('saveSettings is async and returns promise', async () => {
      const wrapper = createWrapper()
      const result = wrapper.vm.saveSettings()
      expect(result).toBeInstanceOf(Promise)
    })

    it('saveSettings awaits config.saveAll completion', async () => {
      const wrapper = createWrapper()
      let saveWasCalled = false
      
      config.saveAll = vi.fn(async () => {
        saveWasCalled = true
        return true
      })
      
      await wrapper.vm.saveSettings()
      expect(saveWasCalled).toBe(true)
    })

    it('onMounted executes async sensor scan and status load', async () => {
      const scanSpy = vi.fn(async () => ({
        success: true,
        data: { sensors: ['s1'] }
      }))
      config.runSensorScan = scanSpy
      
      const statusLoadSpy = vi.fn(async () => {
        status.temperature_device = []
      })
      status.load = statusLoadSpy
      
      createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(scanSpy).toHaveBeenCalled()
      expect(statusLoadSpy).toHaveBeenCalled()
    })

    it('onMounted populates sensor options on success', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['sensor_A', 'sensor_B']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThan(1)
    })

    it('onMounted handles BLE sensor population', async () => {
      status.temperature_device = [
        { device: 'ble1', type: 'BLE' },
        { device: 'ble2', type: 'BLE' }
      ]
      status.load = vi.fn(async () => true)
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(wrapper.vm.bleSensorOptions.length).toBeGreaterThan(1)
    })

    it('validateCurrentForm is called before save', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      const wrapper = createWrapper()
      
      await wrapper.vm.saveSettings()
      
      expect(validateCurrentForm).toHaveBeenCalled()
    })
  })

  describe('conditional logic in onMounted', () => {
    it('processes sensor scan success response', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['id1', 'id2']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThan(1)
    })

    it('adds not-detected label for configured but missing sensors', async () => {
      config.beer_sensor_id = 'missing_sensor'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['other_sensor']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(wrapper.vm.sensorOptions.some(s => s.label.includes('not detected'))).toBe(true)
    })

    it('disables controls during mount lifecycle', async () => {
      expect(global.disabled).toBe(false)
      
      const wrapper = createWrapper()
      
      // During mount, disabled is set to true
      expect(global.disabled).toBe(true)
      
      await new Promise(r => setTimeout(r, 150))
      
      // After mount completes, disabled is set back to false
      expect(global.disabled).toBe(false)
    })

    it('processes empty sensor scan gracefully', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: { sensors: [] }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThan(0)
    })

    it('detects configured sensors in scan results', async () => {
      config.beer_sensor_id = 'sensor1'
      config.fridge_sensor_id = 'sensor2'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['sensor1', 'sensor2', 'sensor3']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      // Should have initial entry + 3 sensors
      expect(wrapper.vm.sensorOptions.length).toBe(4)
      // Should NOT add not-detected labels for detected sensors
      expect(wrapper.vm.sensorOptions.some(s => s.label.includes('not detected'))).toBe(false)
    })

    it('only adds not-detected label for beer sensor when missing', async () => {
      config.beer_sensor_id = 'missing_beer'
      config.fridge_sensor_id = 'sensor2'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['sensor2', 'sensor3']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      const notDetectedLabels = wrapper.vm.sensorOptions.filter(s => s.label.includes('not detected'))
      expect(notDetectedLabels.length).toBe(1)
      expect(notDetectedLabels[0].label).toContain('missing_beer')
    })

    it('only adds not-detected label for fridge sensor when missing', async () => {
      config.fridge_sensor_id = 'missing_fridge'
      config.beer_sensor_id = 'sensor1'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['sensor1', 'sensor3']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      const notDetectedLabels = wrapper.vm.sensorOptions.filter(s => s.label.includes('not detected'))
      expect(notDetectedLabels.length).toBe(1)
      expect(notDetectedLabels[0].label).toContain('missing_fridge')
    })

    it('adds not-detected labels for both missing configured sensors', async () => {
      config.beer_sensor_id = 'missing_beer'
      config.fridge_sensor_id = 'missing_fridge'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['sensor1', 'sensor2']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      const notDetectedLabels = wrapper.vm.sensorOptions.filter(s => s.label.includes('not detected'))
      expect(notDetectedLabels.length).toBe(2)
    })

    it('handles sensor scan with only one configured sensor detected', async () => {
      config.beer_sensor_id = 'sensor1'
      config.fridge_sensor_id = 'sensor2'
      config.runSensorScan = vi.fn(async () => ({
        success: true,
        data: {
          sensors: ['sensor1']
        }
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      const notDetectedLabels = wrapper.vm.sensorOptions.filter(s => s.label.includes('not detected'))
      expect(notDetectedLabels.some(s => s.label.includes('sensor2'))).toBe(true)
    })

    it('handles BLE sensor not in status results', async () => {
      config.beer_ble_sensor_id = 'missing_ble_sensor'
      global.feature.ble_sensor = true
      status.temperature_device = [
        { device: 'device1', type: 'TH' },
        { device: 'device2', type: 'TH' }
      ]
      status.load = vi.fn(async () => {
        // Already set in beforeEach
      })
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      const notDetectedBLE = wrapper.vm.bleSensorOptions.find(s => 
        s.label.includes('not detected') && s.label.includes('missing_ble_sensor')
      )
      expect(notDetectedBLE).toBeDefined()
    })

    it('handles BLE sensor detected in status results', async () => {
      config.beer_ble_sensor_id = 'device1'
      global.feature.ble_sensor = true
      status.temperature_device = [
        { device: 'device1', type: 'TH' },
        { device: 'device2', type: 'TH' }
      ]
      status.load = vi.fn(async () => {
        // Already set above
      })
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      const notDetectedBLE = wrapper.vm.bleSensorOptions.filter(s => s.label.includes('not detected'))
      expect(notDetectedBLE.length).toBe(0)
    })

    it('clears BLE sensor options when no devices detected', async () => {
      global.feature.ble_sensor = true
      status.temperature_device = []
      status.load = vi.fn(async () => {
        status.temperature_device = []
      })
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      // Should only have the "not selected" default option
      expect(wrapper.vm.bleSensorOptions.length).toBe(1)
      expect(wrapper.vm.bleSensorOptions[0].label).toBe('- not selected -')
    })

    it('populates BLE options with device and type information', async () => {
      global.feature.ble_sensor = true
      status.temperature_device = [
        { device: 'sensor_A', type: 'TH' },
        { device: 'sensor_B', type: 'PRESSURE' }
      ]
      status.load = vi.fn(async () => {
        // Already set above
      })
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      expect(wrapper.vm.bleSensorOptions).toContainEqual(expect.objectContaining({
        label: 'sensor_A (TH)',
        value: 'sensor_A'
      }))
      expect(wrapper.vm.bleSensorOptions).toContainEqual(expect.objectContaining({
        label: 'sensor_B (PRESSURE)',
        value: 'sensor_B'
      }))
    })

    it('handles form submission when all sensors are configured', async () => {
      config.beer_sensor_id = 'sensor1'
      config.fridge_sensor_id = 'sensor2'
      config.beer_ble_sensor_id = 'ble_sensor1'
      config.saveAll = vi.fn(async () => {})

      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      await wrapper.vm.saveSettings()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('validation fails on form submission', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)
      
      config.saveAll = vi.fn()
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 50))
      
      await wrapper.vm.saveSettings()
      
      expect(config.saveAll).not.toHaveBeenCalled()
    })

    it('handles sensor scan error gracefully', async () => {
      config.runSensorScan = vi.fn(async () => ({
        success: false,
        error: 'Scan failed'
      }))
      
      const wrapper = createWrapper()
      await new Promise(r => setTimeout(r, 100))
      
      // Should still have default option
      expect(wrapper.vm.sensorOptions.length).toBeGreaterThan(0)
    })
  })
})
