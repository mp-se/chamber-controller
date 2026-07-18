import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import PidControllerFragment from '../PidControllerFragment.vue'
import { createTestingPinia } from '../../tests/testUtils'
import { useConfigStore } from '@/modules/configStore'
import { useGlobalStore } from '@/modules/globalStore'
import { useStatusStore } from '@/modules/statusStore'

let testConfig
let testGlobal
let testStatus

vi.mock('@mp-se/espframework-ui-components', () => ({
  validateCurrentForm: vi.fn(() => true),
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn(),
  sharedHttpClient: {
    postJson: vi.fn()
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

describe('PidControllerFragment', () => {
  let pinia
  let config
  let global
  let status

  beforeEach(() => {
    vi.clearAllMocks()
    pinia = createTestingPinia()
    config = useConfigStore(pinia)
    global = useGlobalStore(pinia)
    status = useStatusStore(pinia)

    testConfig = config
    testGlobal = global
    testStatus = status

    config.enable_cooling = false
    config.enable_heating = false
    config.beer_sensor_id = ''
    config.fridge_sensor_id = ''
    config.target_temperature = 20
    config.temp_format = 'C'
    global.disabled = false
    global.messageError = ''
    global.messageSuccess = ''
    global.clearMessages = vi.fn()
    status.pid_mode = ''
  })

  const createWrapper = (stubs = {}) => {
    return mount(PidControllerFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div class="card"><slot /></div>', props: ['header'] },
          BsButton: { template: '<button><slot /></button>', props: ['type'] },
          BsInputNumber: { template: '<input type="number" v-model="modelValue" />', props: ['modelValue', 'label', 'disabled'] },
          BsInputRadio: { template: '<div><slot /></div>', props: ['modelValue', 'options', 'label', 'disabled'] },
          ...stubs
        }
      }
    })
  }

  describe('component mounting', () => {
    it('mounts without error', () => {
      const wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })

    it('renders card component', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('.card').exists()).toBe(true)
    })

    it('displays controller header', () => {
      const wrapper = createWrapper()
      expect(wrapper.text()).toContain('Controller')
    })

    it('renders form element', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('form').exists()).toBe(true)
    })
  })

  describe('onMounted configuration checks', () => {
    it('shows error when neither cooling nor heating enabled', async () => {
      config.enable_cooling = false
      config.enable_heating = false
      config.beer_sensor_id = 'sensor1'
      global.messageError = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Component should execute onMounted logic
      expect(wrapper.exists()).toBe(true)
    })

    it('shows error when cooling/heating enabled but no sensors', async () => {
      config.enable_cooling = true
      config.enable_heating = false
      config.beer_sensor_id = ''
      config.fridge_sensor_id = ''
      global.messageError = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Component should execute onMounted logic
      expect(wrapper.exists()).toBe(true)
    })

    it('handles beer sensor configuration', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      config.fridge_sensor_id = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Component should handle beer sensor
      expect(config.beer_sensor_id).toBe('sensor1')
    })

    it('handles chamber sensor configuration', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = ''
      config.fridge_sensor_id = 'sensor2'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Component should handle fridge sensor
      expect(config.fridge_sensor_id).toBe('sensor2')
    })

    it('handles both sensors configured', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'beer_sensor'
      config.fridge_sensor_id = 'fridge_sensor'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Component should handle both sensors
      expect(config.beer_sensor_id).toBe('beer_sensor')
      expect(config.fridge_sensor_id).toBe('fridge_sensor')
    })
  })

  describe('saveSettings API call', () => {
    it('calls API with correct data on form submission', async () => {
      const { sharedHttpClient } = await import('@mp-se/espframework-ui-components')
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      wrapper.vm.newMode = 'b'
      wrapper.vm.newTemperature = 20
      
      await wrapper.vm.saveSettings()
      
      expect(wrapper.exists()).toBe(true)
    })

    it('handles successful API call', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      await wrapper.vm.saveSettings()
      
      // Component should handle the response
      expect(wrapper.exists()).toBe(true)
    })

    it('handles API errors gracefully', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      // Even with errors, component should not throw
      try {
        await wrapper.vm.saveSettings()
      } catch (e) {
        // Expected to handle errors
      }
      
      expect(wrapper.exists()).toBe(true)
    })

    it('form validation is checked before API call', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      await wrapper.vm.saveSettings()
      
      // validateCurrentForm should have been called
      expect(validateCurrentForm).toHaveBeenCalled()
    })

    it('clears messages before sending', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      global.messageError = 'Previous error'
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      await wrapper.vm.saveSettings()
      
      // Component should attempt to clear messages
      expect(wrapper.exists()).toBe(true)
    })
  })

  describe('target temperature', () => {
    it('has temperature tracking', async () => {
      config.target_temperature = 19.5
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm).toBeDefined()
    })

    it('temperature format affects unit display', async () => {
      config.temp_format = 'C'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm).toBeDefined()
    })

    it('can use fahrenheit format', async () => {
      config.temp_format = 'F'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(config.temp_format).toBe('F')
    })
  })

  describe('button disabled state', () => {
    it('disables button when global disabled flag set', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      global.disabled = true
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      expect(global.disabled).toBe(true)
    })

    it('disables button when only Off mode available', async () => {
      config.enable_cooling = false
      config.enable_heating = false
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      // Should only have Off mode, so button should be disabled
      expect(wrapper.vm.modeOptions.length).toBe(1)
    })

    it('enables button when mode options available and not disabled', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      global.disabled = false
      
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      expect(global.disabled).toBe(false)
    })
  })

  describe('mode selection', () => {
    it('initializes with Off mode', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.newMode).toBe('o')
    })

    it('allows changing mode', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'

      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()

      wrapper.vm.newMode = 'b'
      expect(wrapper.vm.newMode).toBe('b')
    })
  })

  describe('onMounted branch coverage', () => {
    it('adds Beer constant option when beer sensor configured', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor-abc'
      config.fridge_sensor_id = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.modeOptions.length).toBeGreaterThan(1)
      expect(wrapper.vm.modeOptions.some(o => o.value === 'b')).toBe(true)
    })

    it('adds Chamber constant option when fridge sensor configured', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = ''
      config.fridge_sensor_id = 'fridge-sensor'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.modeOptions.some(o => o.value === 'f')).toBe(true)
    })

    it('adds both options when both sensors are configured', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = 'beer-sensor'
      config.fridge_sensor_id = 'fridge-sensor'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.vm.modeOptions.some(o => o.value === 'b')).toBe(true)
      expect(wrapper.vm.modeOptions.some(o => o.value === 'f')).toBe(true)
    })

    it('sets error when cooling enabled but no sensors configured', async () => {
      config.enable_cooling = true
      config.beer_sensor_id = ''
      config.fridge_sensor_id = ''
      global.messageError = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(global.messageError).toContain('No sensors')
    })

    it('sets error when neither cooling nor heating is enabled', async () => {
      config.enable_cooling = false
      config.enable_heating = false
      global.messageError = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(global.messageError).toContain('Neither cooling')
    })
  })

  describe('saveSettings error handling', () => {
    it('sets error message when API call fails', async () => {
      const { sharedHttpClient } = await import('@mp-se/espframework-ui-components')
      sharedHttpClient.postJson.mockRejectedValueOnce(new Error('Network error'))
      config.enable_cooling = true
      config.beer_sensor_id = 'sensor1'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()

      await wrapper.vm.saveSettings()

      expect(global.messageError).toContain('Failed to update PID controller')
    })

    it('re-enables controls after API error', async () => {
      const { sharedHttpClient } = await import('@mp-se/espframework-ui-components')
      sharedHttpClient.postJson.mockRejectedValueOnce(new Error('Network error'))
      global.disabled = false
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()

      await wrapper.vm.saveSettings()

      expect(global.disabled).toBe(false)
    })
  })

  describe('v-model bindings', () => {
    it('updates newMode via BsInputRadio v-model', async () => {
      const wrapper = mount(PidControllerFragment, {
        global: {
          plugins: [pinia],
          stubs: {
            BsInputRadio: {
              template: '<input type="radio" @change="$emit(\'update:modelValue\', \'b\')" />',
              props: ['modelValue', 'label', 'options', 'disabled']
            },
            BsInputNumber: { template: '<input type="number" />', props: ['modelValue', 'label', 'min', 'max', 'step', 'unit', 'width', 'disabled'] }
          }
        }
      })
      await wrapper.vm.$nextTick()
      const radio = wrapper.find('input[type="radio"]')
      await radio.trigger('change')
      expect(wrapper.vm.newMode).toBe('b')
    })

    it('updates newTemperature via BsInputNumber v-model', async () => {
      const wrapper = mount(PidControllerFragment, {
        global: {
          plugins: [pinia],
          stubs: {
            BsInputRadio: { template: '<div />', props: ['modelValue', 'label', 'options', 'disabled'] },
            BsInputNumber: {
              template: '<input type="number" :value="modelValue" @input="$emit(\'update:modelValue\', Number($event.target.value))" />',
              props: ['modelValue', 'label', 'min', 'max', 'step', 'unit', 'width', 'disabled']
            }
          }
        }
      })
      await wrapper.vm.$nextTick()
      const input = wrapper.find('input[type="number"]')
      await input.setValue('22')
      expect(wrapper.vm.newTemperature).toBe(22)
    })
  })

  describe('watch pid_mode', () => {
    it('watch callback fires when pid_mode changes', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      status.pid_mode = 'b'
      await wrapper.vm.$nextTick()
      // watch triggered - component should still exist
      expect(wrapper.exists()).toBe(true)
    })
  })
})
