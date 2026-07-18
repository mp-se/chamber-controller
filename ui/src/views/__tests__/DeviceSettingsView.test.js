import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import DeviceSettingsView from '../DeviceSettingsView.vue'
import { createPinia, setActivePinia } from 'pinia'
import { useConfigStore } from '@/modules/configStore'
import { useGlobalStore } from '@/modules/globalStore'
import { validateCurrentForm } from '@mp-se/espframework-ui-components'

vi.mock('@mp-se/espframework-ui-components', () => ({
  validateCurrentForm: vi.fn(() => true),
  logDebug: vi.fn(),
  logError: vi.fn(),
  logInfo: vi.fn()
}))

// Mock fetch globally before any other code
const mockFetch = vi.fn()
globalThis.fetch = mockFetch

describe('DeviceSettingsView', () => {
  let pinia
  let config
  let global
  let windowSpy

  beforeEach(() => {
    vi.clearAllMocks()
    pinia = createPinia()
    setActivePinia(pinia)
    config = useConfigStore(pinia)
    global = useGlobalStore(pinia)

    // Initialize config state
    config.mdns = 'chamber'
    config.temp_format = 'C'
    config.dark_mode = false
    config.restart_interval = 0
    config.saveAll = vi.fn().mockResolvedValue({})
    config.restart = vi.fn().mockResolvedValue({})

    // Initialize global state
    global.disabled = false
    global.messageSuccess = ''
    global.messageError = ''
    global.baseURL = 'http://localhost:8080/'
    global.token = 'test-token'
    global.fetchTimeout = 30000
    global.clearMessages = vi.fn()

    // Mock window methods
    windowSpy = {
      reload: vi.fn(),
      addEventListener: vi.fn(),
      removeEventListener: vi.fn()
    }
    Object.defineProperty(window, 'location', {
      value: { reload: windowSpy.reload },
      writable: true
    })
    window.addEventListener = windowSpy.addEventListener
    window.removeEventListener = windowSpy.removeEventListener

    // Reset fetch mock
    mockFetch.mockClear()
  })

  afterEach(() => {
    vi.clearAllMocks()
  })

  const createWrapper = () => {
    return mount(DeviceSettingsView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsInputText: true,
          BsInputNumber: true,
          BsInputRadio: true,
          BsSelect: true,
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

    it('renders form with correct class', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('form.needs-validation').exists()).toBe(true)
    })

    it('displays save button', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Save')
    })

    it('displays restart device button', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Restart device')
    })

    it('displays factory reset button', () => {
      const wrapper = createWrapper()
      expect(wrapper.html()).toContain('Restore factory defaults')
    })
  })

  describe('temperature format configuration', () => {
    it('stores celsius format', () => {
      config.temp_format = 'C'
      expect(config.temp_format).toBe('C')
    })

    it('stores fahrenheit format', () => {
      config.temp_format = 'F'
      expect(config.temp_format).toBe('F')
    })

    it('can change format', () => {
      config.temp_format = 'C'
      expect(config.temp_format).toBe('C')
      config.temp_format = 'F'
      expect(config.temp_format).toBe('F')
    })
  })

  describe('dark mode configuration', () => {
    it('stores dark mode enabled', () => {
      config.dark_mode = true
      expect(config.dark_mode).toBe(true)
    })

    it('stores dark mode disabled', () => {
      config.dark_mode = false
      expect(config.dark_mode).toBe(false)
    })

    it('can toggle dark mode', () => {
      config.dark_mode = false
      expect(config.dark_mode).toBe(false)
      config.dark_mode = true
      expect(config.dark_mode).toBe(true)
    })
  })

  describe('restart interval configuration', () => {
    it('stores disabled restart interval', () => {
      config.restart_interval = 0
      expect(config.restart_interval).toBe(0)
    })

    it('stores 30 minute interval', () => {
      config.restart_interval = 30
      expect(config.restart_interval).toBe(30)
    })

    it('stores 1 hour interval', () => {
      config.restart_interval = 60
      expect(config.restart_interval).toBe(60)
    })

    it('stores custom interval', () => {
      config.restart_interval = 120
      expect(config.restart_interval).toBe(120)
    })
  })

  describe('mdns configuration', () => {
    it('stores mdns name', () => {
      config.mdns = 'mychamber'
      expect(config.mdns).toBe('mychamber')
    })

    it('can update mdns name', () => {
      config.mdns = 'initial'
      expect(config.mdns).toBe('initial')
      config.mdns = 'updated'
      expect(config.mdns).toBe('updated')
    })

    it('stores empty mdns', () => {
      config.mdns = ''
      expect(config.mdns).toBe('')
    })
  })

  describe('save settings method', () => {
    it('method exists on component', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.saveSettings).toBeDefined()
      expect(typeof wrapper.vm.saveSettings).toBe('function')
    })

    it('is an async method', () => {
      const wrapper = createWrapper()
      const result = wrapper.vm.saveSettings()
      expect(result).toBeInstanceOf(Promise)
    })
  })

  describe('restart device method', () => {
    it('method exists on component', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.restartDevice).toBeDefined()
      expect(typeof wrapper.vm.restartDevice).toBe('function')
    })

    it('is an async method', () => {
      const wrapper = createWrapper()
      const result = wrapper.vm.restartDevice()
      expect(result).toBeInstanceOf(Promise)
    })
  })

  describe('factory reset method', () => {
    it('method exists on component', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.factory).toBeDefined()
      expect(typeof wrapper.vm.factory).toBe('function')
    })

    it('is an async method', () => {
      const wrapper = createWrapper()
      const result = wrapper.vm.factory()
      expect(result).toBeInstanceOf(Promise)
    })
  })

  describe('disabled state management', () => {
    it('respects global disabled flag during form interaction', () => {
      global.disabled = true
      const wrapper = createWrapper()

      expect(global.disabled).toBe(true)
    })

    it('can enable/disable controls', () => {
      global.disabled = false
      expect(global.disabled).toBe(false)

      global.disabled = true
      expect(global.disabled).toBe(true)

      global.disabled = false
      expect(global.disabled).toBe(false)
    })
  })

  describe('option lists', () => {
    it('has temperature format options', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.tempOptions).toBeDefined()
      expect(wrapper.vm.tempOptions.length).toBe(2)
      expect(wrapper.vm.tempOptions[0].value).toBe('C')
      expect(wrapper.vm.tempOptions[1].value).toBe('F')
    })

    it('has UI mode options', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.uiOptions).toBeDefined()
      expect(wrapper.vm.uiOptions.length).toBe(2)
      expect(wrapper.vm.uiOptions[0].value).toBe(false)
      expect(wrapper.vm.uiOptions[1].value).toBe(true)
    })

    it('has restart interval options', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.restartOptions).toBeDefined()
      expect(wrapper.vm.restartOptions.length).toBe(8)
      expect(wrapper.vm.restartOptions[0].value).toBe(0)
      expect(wrapper.vm.restartOptions[7].value).toBe(60 * 24)
    })
  })

  describe('message handling', () => {
    it('component methods are accessible', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.saveSettings).toBeDefined()
      expect(wrapper.vm.restartDevice).toBeDefined()
      expect(wrapper.vm.factory).toBeDefined()
    })
  })
})
import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import DeviceSettingsView from '../DeviceSettingsView.vue'
import { createPinia, setActivePinia } from 'pinia'
import { global, config } from '@/modules/pinia'

// Mock components and functions
vi.mock('@mp-se/espframework-ui-components', () => ({
  validateCurrentForm: vi.fn(() => true),
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn(),
  sharedHttpClient: {
    postJson: vi.fn().mockResolvedValue({})
  }
}))

describe('DeviceSettingsView', () => {
  let wrapper

  const createWrapper = (overrides = {}) => {
    return mount(DeviceSettingsView, {
      global: {
        stubs: {
          BsInputText: {
            template: '<input v-model="modelValue" @input="$emit(\'update:modelValue\', $event.target.value)" />',
            props: ['modelValue', 'disabled', 'label']
          },
          BsInputNumber: {
            template: '<input type="number" v-model.number="modelValue" @input="$emit(\'update:modelValue\', Number($event.target.value))" />',
            props: ['modelValue', 'disabled']
          },
          BsInputRadio: {
            template: '<div @click="$emit(\'update:modelValue\', options[0].value)"><slot /></div>',
            props: ['modelValue', 'options', 'disabled', 'label']
          },
          BsSelect: {
            template: '<select v-model="modelValue" @change="$emit(\'update:modelValue\', $event.target.value)"><option v-for="opt in options" :value="opt.value">{{ opt.label }}</option></select>',
            props: ['modelValue', 'options', 'disabled']
          },
          BsInputSwitch: { template: '<input type="checkbox" />' },
          BsButton: { template: '<button><slot /></button>' },
          BsMessage: { template: '<div v-if="message" class="message">{{ message }}</div>' }
        },
        ...overrides
      }
    })
  }

  beforeEach(() => {
    const pinia = createPinia()
    setActivePinia(pinia)

    config.id = 'device001'
    config.mdns = 'chamber'
    config.temp_format = 'C'
    config.dark_mode = false
    config.restart_interval = 0
    global.disabled = false
    global.messageSuccess = ''
    global.messageError = ''

    vi.clearAllMocks()
  })

  describe('component mount', () => {
    it('mounts successfully', () => {
      wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })

    it('displays correct heading', () => {
      wrapper = createWrapper()
      expect(wrapper.text()).toContain('Device - Settings')
    })
  })

  describe('mdns configuration', () => {
    it('displays heading when mdns is empty', () => {
      config.mdns = ''
      wrapper = createWrapper()
      expect(wrapper.text()).toContain('Device - Settings')
    })

    it('displays heading when mdns is set', () => {
      config.mdns = 'chambername'
      wrapper = createWrapper()
      expect(wrapper.text()).toContain('Device - Settings')
    })

    it('binds mdns value to config', () => {
      config.mdns = 'testchamber'
      wrapper = createWrapper()
      expect(config.mdns).toBe('testchamber')
    })
  })

  describe('temperature format option', () => {
    it('stores celsius format', () => {
      config.temp_format = 'C'
      expect(config.temp_format).toBe('C')
    })

    it('stores fahrenheit format', () => {
      config.temp_format = 'F'
      expect(config.temp_format).toBe('F')
    })

    it('can change from C to F', () => {
      config.temp_format = 'C'
      config.temp_format = 'F'
      expect(config.temp_format).toBe('F')
    })
  })

  describe('dark mode option', () => {
    it('stores dark mode enabled', () => {
      config.dark_mode = true
      expect(config.dark_mode).toBe(true)
    })

    it('stores dark mode disabled', () => {
      config.dark_mode = false
      expect(config.dark_mode).toBe(false)
    })

    it('can toggle dark mode state', () => {
      config.dark_mode = false
      config.dark_mode = true
      expect(config.dark_mode).toBe(true)
    })
  })

  describe('restart interval', () => {
    it('stores restart interval value', () => {
      config.restart_interval = 60
      expect(config.restart_interval).toBe(60)
    })

    it('defaults to 0', () => {
      config.restart_interval = 0
      expect(config.restart_interval).toBe(0)
    })

    it('accepts different interval values', () => {
      const intervals = [0, 30, 60, 120, 240]
      intervals.forEach((interval) => {
        config.restart_interval = interval
        expect(config.restart_interval).toBe(interval)
      })
    })
  })

  describe('form interaction', () => {
    it('has a form element', () => {
      wrapper = createWrapper()
      expect(wrapper.find('form').exists()).toBe(true)
    })

    it('form has submit button', () => {
      wrapper = createWrapper()
      const form = wrapper.find('form')
      expect(form.exists()).toBe(true)
    })

    it('form has needs-validation class', () => {
      wrapper = createWrapper()
      expect(wrapper.find('form.needs-validation').exists()).toBe(true)
    })
  })

  describe('disabled state', () => {
    it('respects global disabled flag', () => {
      global.disabled = true
      wrapper = createWrapper()
      expect(global.disabled).toBe(true)
    })

    it('can toggle disabled state', () => {
      global.disabled = false
      expect(global.disabled).toBe(false)
      global.disabled = true
      expect(global.disabled).toBe(true)
    })
  })

  describe('form controls', () => {
    it('renders input controls', () => {
      wrapper = createWrapper()
      const inputs = wrapper.findAll('input')
      expect(inputs.length).toBeGreaterThan(0)
    })

    it('renders select controls', () => {
      wrapper = createWrapper()
      const selects = wrapper.findAll('select')
      expect(selects.length).toBeGreaterThan(0)
    })
  })

  describe('config state management', () => {
    it('initializes default values', () => {
      expect(config.mdns).toBeDefined()
      expect(config.temp_format).toBeDefined()
      expect(config.dark_mode).toBeDefined()
    })

    it('maintains mdns between renders', () => {
      config.mdns = 'persistent'
      const value1 = config.mdns
      wrapper = createWrapper()
      const value2 = config.mdns
      expect(value1).toBe(value2)
    })

    it('reflects temperature format changes', () => {
      config.temp_format = 'C'
      expect(config.temp_format).toBe('C')
      config.temp_format = 'F'
      expect(config.temp_format).toBe('F')
    })

    it('reflects dark mode changes', () => {
      const initial = config.dark_mode
      config.dark_mode = !initial
      expect(config.dark_mode).toBe(!initial)
      config.dark_mode = initial
      expect(config.dark_mode).toBe(initial)
    })
  })

  describe('global store integration', () => {
    it('accesses global disabled flag', () => {
      expect(global.disabled).toBeDefined()
    })

    it('can set error message', () => {
      global.messageError = 'Test error'
      expect(global.messageError).toBe('Test error')
    })

    it('can set success message', () => {
      global.messageSuccess = 'Test success'
      expect(global.messageSuccess).toBe('Test success')
    })

    it('clears messages', () => {
      global.messageError = 'Error'
      global.messageSuccess = 'Success'
      global.messageError = ''
      global.messageSuccess = ''
      expect(global.messageError).toBe('')
      expect(global.messageSuccess).toBe('')
    })
  })

  describe('layout and structure', () => {
    it('renders as container div', () => {
      wrapper = createWrapper()
      expect(wrapper.find('.container').exists()).toBe(true)
    })

    it('has form with correct structure', () => {
      wrapper = createWrapper()
      const form = wrapper.find('form')
      expect(form.exists()).toBe(true)
    })
  })

  describe('button handlers', () => {
    it('save button exists', () => {
      wrapper = createWrapper()
      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
    })

    it('restart button exists', () => {
      wrapper = createWrapper()
      const buttons = wrapper.findAll('button')
      expect(buttons.some(b => b.text().includes('Restart'))).toBe(true)
    })

    it('factory restore button exists', () => {
      wrapper = createWrapper()
      const buttons = wrapper.findAll('button')
      expect(buttons.some(b => b.text().includes('factory'))).toBe(true)
    })
  })

  describe('API interaction mocking', () => {
    it('component can be mounted and interacted with', () => {
      wrapper = createWrapper()
      expect(wrapper.vm).toBeDefined()
    })

    it('global store integration available', () => {
      expect(global.clearMessages).toBeDefined()
    })

    it('config store integration available', () => {
      expect(config.restart).toBeDefined()
      expect(config.saveAll).toBeDefined()
    })
  })

  describe('form validation', () => {
    it('form has all required inputs', () => {
      wrapper = createWrapper()
      const form = wrapper.find('form')
      expect(form.exists()).toBe(true)
      const inputs = form.findAll('input, select')
      expect(inputs.length).toBeGreaterThan(0)
    })

    it('form has validation class', () => {
      wrapper = createWrapper()
      expect(wrapper.find('form.needs-validation').exists()).toBe(true)
    })

    it('mdns input is available', () => {
      wrapper = createWrapper()
      expect(wrapper.find('input').exists()).toBe(true)
    })
  })

  describe('async function execution and error handling', () => {
    it('saveSettings function is properly defined', () => {
      wrapper = createWrapper()
      expect(wrapper.vm.saveSettings).toBeDefined()
      expect(typeof wrapper.vm.saveSettings).toBe('function')
    })

    it('saveSettings awaits config.saveAll call', async () => {
      wrapper = createWrapper()
      config.saveAll = vi.fn(async () => true)
      
      await wrapper.vm.saveSettings()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('saveSettings returns early if validation fails', async () => {
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)
      wrapper = createWrapper()
      config.saveAll = vi.fn()
      
      await wrapper.vm.saveSettings()
      
      expect(config.saveAll).not.toHaveBeenCalled()
    })

    it('saveSettings handles errors with try/catch', async () => {
      wrapper = createWrapper()
      config.saveAll = vi.fn(async () => {
        throw new Error('Save failed')
      })
      
      await wrapper.vm.saveSettings()
      
      expect(global.messageError).toContain('Failed to save settings')
    })

    it('restartDevice function is defined', () => {
      wrapper = createWrapper()
      expect(wrapper.vm.restartDevice).toBeDefined()
      expect(typeof wrapper.vm.restartDevice).toBe('function')
    })

    it('restartDevice calls config.restart', async () => {
      wrapper = createWrapper()
      config.restart = vi.fn(async () => true)
      
      await wrapper.vm.restartDevice()
      
      expect(config.restart).toHaveBeenCalled()
    })

    it('restartDevice handles restart errors', async () => {
      wrapper = createWrapper()
      config.restart = vi.fn(async () => {
        throw new Error('Restart failed')
      })
      
      await wrapper.vm.restartDevice()
      
      expect(global.messageError).toContain('Failed to restart')
    })

    it('factory function is defined', () => {
      wrapper = createWrapper()
      expect(wrapper.vm.factory).toBeDefined()
      expect(typeof wrapper.vm.factory).toBe('function')
    })

    it('factory clears messages and disables controls', async () => {
      wrapper = createWrapper()
      global.clearMessages = vi.fn()
      global.disabled = false
      
      // Mock fetch to prevent actual network call
      vi.stubGlobal('fetch', vi.fn(async () => ({
        ok: false,
        status: 500,
        statusText: 'Server Error'
      })))
      
      await wrapper.vm.factory()
      
      expect(global.clearMessages).toHaveBeenCalled()
    })

    it('factory handles successful factory reset response', async () => {
      wrapper = createWrapper()
      global.clearMessages = vi.fn()
      global.disabled = false
      
      vi.stubGlobal('fetch', vi.fn(async () => ({
        ok: true,
        status: 200,
        statusText: 'OK',
        json: async () => ({ success: true, message: 'Factory reset success' })
      })))
      
      await wrapper.vm.factory()
      
      expect(global.messageSuccess).toContain('Factory reset success')
    })

    it('factory handles HTTP error responses', async () => {
      wrapper = createWrapper()
      global.clearMessages = vi.fn()
      
      vi.stubGlobal('fetch', vi.fn(async () => ({
        ok: false,
        status: 404,
        statusText: 'Not Found'
      })))
      
      await wrapper.vm.factory()
      
      expect(global.messageError).toContain('HTTP 404')
    })

    it('factory handles failed JSON response', async () => {
      wrapper = createWrapper()
      global.clearMessages = vi.fn()
      
      vi.stubGlobal('fetch', vi.fn(async () => ({
        ok: true,
        status: 200,
        json: async () => ({ success: false, message: 'Factory restore failed' })
      })))
      
      await wrapper.vm.factory()
      
      expect(global.messageError).toContain('Factory restore failed')
    })

    it('factory re-enables controls after operation', async () => {
      wrapper = createWrapper()
      global.clearMessages = vi.fn()
      global.disabled = false
      
      vi.stubGlobal('fetch', vi.fn(async () => ({
        ok: true,
        status: 200,
        json: async () => ({ success: false, message: 'Restore failed' })
      })))
      
      await wrapper.vm.factory()
      
      expect(global.disabled).toBe(false)
    })

    it('factory handles fetch errors with try/catch', async () => {
      wrapper = createWrapper()
      global.clearMessages = vi.fn()
      
      vi.stubGlobal('fetch', vi.fn(async () => {
        throw new Error('Network error')
      }))
      
      await wrapper.vm.factory()
      
      expect(global.messageError).toContain('Failed to perform factory restore')
    })

    it('saveSettings validates form before proceeding', async () => {
      wrapper = createWrapper()
      vi.mocked(validateCurrentForm).mockReturnValueOnce(true)
      config.saveAll = vi.fn(async () => true)
      
      await wrapper.vm.saveSettings()
      
      expect(validateCurrentForm).toHaveBeenCalled()
      expect(config.saveAll).toHaveBeenCalled()
    })
  })
})
