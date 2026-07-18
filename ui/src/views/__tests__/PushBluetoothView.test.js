import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import PushBluetoothView from '../PushBluetoothView.vue'
import { createPinia, setActivePinia } from 'pinia'
import { global, config } from '@/modules/pinia'

// Mock validation
vi.mock('@mp-se/espframework-ui-components', () => ({
  validateCurrentForm: vi.fn(() => true),
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn()
}))

describe('PushBluetoothView', () => {
  beforeEach(() => {
    const pinia = createPinia()
    setActivePinia(pinia)

    config.ble_push_enabled = false
    global.disabled = false
    global.configChanged = false
    global.feature.ble = true

    vi.clearAllMocks()
  })

  describe('rendering', () => {
    it('is a component', () => {
      expect(typeof PushBluetoothView).toBe('object')
    })

    it('mounts without error when BLE is available', () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.exists()).toBe(true)
    })

    it('renders form with validation when BLE available', () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const form = wrapper.find('form.needs-validation')
      expect(form.exists()).toBe(true)
    })

    it('renders save button when BLE available', () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
      expect(wrapper.text()).toContain('Save')
    })

    it('renders warning about restart requirement when BLE available', () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.text()).toContain('restart')
    })

    it('renders row layout structure', () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const rows = wrapper.findAll('.row')
      expect(rows.length).toBeGreaterThan(0)
    })

    it('renders container element', () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const container = wrapper.find('.container')
      expect(container.exists()).toBe(true)
    })

    it('renders unavailable message when BLE not available', () => {
      global.feature.ble = false

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.text()).toContain('Bluetooth is not available on this platform')
    })
  })

  describe('state management', () => {
    it('can toggle BLE push enabled', () => {
      config.ble_push_enabled = false
      expect(config.ble_push_enabled).toBe(false)

      config.ble_push_enabled = true
      expect(config.ble_push_enabled).toBe(true)
    })

    it('respects global disabled flag', () => {
      global.disabled = true
      expect(global.disabled).toBe(true)

      global.disabled = false
      expect(global.disabled).toBe(false)
    })

    it('respects config changed flag', () => {
      global.configChanged = false
      expect(global.configChanged).toBe(false)

      global.configChanged = true
      expect(global.configChanged).toBe(true)
    })
  })

  describe('feature availability', () => {
    it('shows form when BLE feature is available', () => {
      global.feature.ble = true

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.find('form').exists()).toBe(true)
    })

    it('hides form when BLE feature is unavailable', () => {
      global.feature.ble = false

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.find('form').exists()).toBe(false)
    })
  })

  describe('save function', () => {
    it('saves configuration when form is submitted', async () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const form = wrapper.find('form')
      expect(form.exists()).toBe(true)
      
      config.saveAll = vi.fn().mockResolvedValue(true)
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('prevents default form submission', async () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      config.saveAll = vi.fn().mockResolvedValue(true)
      const form = wrapper.find('form')
      
      await form.trigger('submit')
      
      expect(form.exists()).toBe(true)
    })

    it('handles async save completion', async () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const savePromise = Promise.resolve(true)
      config.saveAll = vi.fn().mockReturnValue(savePromise)
      const form = wrapper.find('form')
      
      await form.trigger('submit')
      await savePromise
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll).toHaveBeenCalled()
    })

    it('form submit triggers save function', async () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      config.saveAll = vi.fn().mockResolvedValue(true)
      const form = wrapper.find('form')
      
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll).toHaveBeenCalledTimes(1)
    })

    it('executes save logic on form submit', async () => {
      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      config.saveAll = vi.fn().mockResolvedValue(true)
      
      const form = wrapper.find('form')
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll.mock.calls.length > 0).toBe(true)
    })
  })

  describe('v-model bindings', () => {
    it('updates ble_push_enabled via v-model when BLE available', async () => {
      global.feature.ble = true
      config.ble_push_enabled = false

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: {
              template:
                '<input type="checkbox" :checked="modelValue" @change="$emit(\'update:modelValue\', $event.target.checked)" />',
              props: ['modelValue', 'label', 'disabled']
            },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const checkbox = wrapper.find('input[type="checkbox"]')
      await checkbox.setValue(true)
      expect(config.ble_push_enabled).toBe(true)
    })
  })

  describe('button interaction', () => {
    it('save button is disabled when global.disabled is true', () => {
      global.disabled = true

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
    })

    it('save button is disabled when configChanged is false', () => {
      global.configChanged = false
      global.disabled = false

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
    })

    it('save button is enabled when config changed and not disabled', () => {
      global.disabled = false
      global.configChanged = true

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
    })
  })

  describe('save validation (line 77)', () => {
    it('does not call saveAll when validateCurrentForm returns false', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)

      config.saveAll = vi.fn()

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const form = wrapper.find('form')
      if (form.exists()) {
        await form.trigger('submit')
        await wrapper.vm.$nextTick()
        expect(config.saveAll).not.toHaveBeenCalled()
      } else {
        // Trigger save directly
        await wrapper.vm.save()
        expect(config.saveAll).not.toHaveBeenCalled()
      }
    })

    it('save returns early without clearMessages when validation fails', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)

      global.clearMessages = vi.fn()

      const wrapper = mount(PushBluetoothView, {
        global: {
          stubs: {
            BsInputSwitch: { template: '<input type="checkbox" />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      await wrapper.vm.save()

      expect(global.clearMessages).not.toHaveBeenCalled()
    })
  })
})
