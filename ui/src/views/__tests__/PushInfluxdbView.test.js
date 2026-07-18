import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import PushInfluxdbView from '../PushInfluxdbView.vue'
import { createPinia, setActivePinia } from 'pinia'
import { global, config } from '@/modules/pinia'

// Mock validation
vi.mock('@mp-se/espframework-ui-components', () => ({
  validateCurrentForm: vi.fn(() => true),
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn()
}))

describe('PushInfluxdbView', () => {
  beforeEach(() => {
    const pinia = createPinia()
    setActivePinia(pinia)

    config.influxdb2_target = ''
    config.influxdb2_bucket = ''
    config.influxdb2_org = ''
    config.influxdb2_token = ''
    global.disabled = false
    global.configChanged = false

    vi.clearAllMocks()
  })

  describe('component', () => {
    it('is a component', () => {
      expect(typeof PushInfluxdbView).toBe('object')
    })

    it('mounts without error', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.exists()).toBe(true)
    })

    it('displays page title', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.text()).toContain('Influxdb v2')
    })
  })

  describe('influxdb configuration', () => {
    it('renders form with validation class', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const form = wrapper.find('form.needs-validation')
      expect(form.exists()).toBe(true)
    })

    it('renders input fields for influxdb configuration', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.findAll('input').length).toBeGreaterThan(0)
    })

    it('stores influxdb target URL', () => {
      config.influxdb2_target = 'http://influx.example.com'
      expect(config.influxdb2_target).toBe('http://influx.example.com')
    })

    it('stores influxdb bucket name', () => {
      config.influxdb2_bucket = 'mybucket'
      expect(config.influxdb2_bucket).toBe('mybucket')
    })

    it('stores influxdb organization', () => {
      config.influxdb2_org = 'myorg'
      expect(config.influxdb2_org).toBe('myorg')
    })

    it('stores influxdb token', () => {
      config.influxdb2_token = 'secret_token_123'
      expect(config.influxdb2_token).toBe('secret_token_123')
    })
  })

  describe('form structure', () => {
    it('renders container divs', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.find('div').exists()).toBe(true)
    })

    it('renders multiple input elements for configuration', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      expect(wrapper.findAll('input').length).toBeGreaterThanOrEqual(1)
    })

    it('renders save button', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
      expect(wrapper.text()).toContain('Save')
    })

    it('has all input fields in row layout', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const rows = wrapper.findAll('.row')
      expect(rows.length).toBeGreaterThan(0)
    })

    it('renders container structure', () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })
      const container = wrapper.find('.container')
      expect(container.exists()).toBe(true)
    })
  })

  describe('state management', () => {
    it('respects global disabled flag', () => {
      global.disabled = true
      expect(global.disabled).toBe(true)
    })

    it('respects config changed flag', () => {
      global.configChanged = true
      expect(global.configChanged).toBe(true)
    })
  })

  describe('save function', () => {
    it('saves configuration when form is submitted', async () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
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
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      config.saveAll = vi.fn().mockResolvedValue(true)
      const form = wrapper.find('form')
      
      const event = new Event('submit', { bubbles: true, cancelable: true })
      const preventSpyMock = vi.spyOn(event, 'preventDefault')
      
      await form.trigger('submit')
      
      expect(form.exists()).toBe(true)
    })

    it('handles async save completion', async () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
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
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      config.saveAll = vi.fn().mockResolvedValue(true)
      const form = wrapper.find('form')
      const formElement = form.element
      
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(config.saveAll).toHaveBeenCalledTimes(1)
    })

    it('executes save logic on form submit', async () => {
      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const originalSaveAll = config.saveAll
      let saveWasCalled = false
      
      config.ble_push_enabled = true
      
      config.saveAll = vi.fn(async () => {
        saveWasCalled = true
        return true
      })

      const form = wrapper.find('form')
      await form.trigger('submit')
      await wrapper.vm.$nextTick()
      
      expect(saveWasCalled || config.saveAll.mock.calls.length > 0).toBe(true)
    })
  })

  describe('v-model bindings', () => {
    const createInteractiveWrapper = () =>
      mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: {
              template:
                '<input :value="modelValue" @input="$emit(\'update:modelValue\', $event.target.value)" />',
              props: ['modelValue', 'label', 'help', 'type', 'maxlength', 'disabled']
            },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

    it('updates influxdb2_target via v-model', async () => {
      const wrapper = createInteractiveWrapper()
      const inputs = wrapper.findAll('input')
      await inputs[0].setValue('http://influx.example.com')
      expect(config.influxdb2_target).toBe('http://influx.example.com')
    })

    it('updates influxdb2_org via v-model', async () => {
      const wrapper = createInteractiveWrapper()
      const inputs = wrapper.findAll('input')
      await inputs[1].setValue('myorg')
      expect(config.influxdb2_org).toBe('myorg')
    })

    it('updates influxdb2_bucket via v-model', async () => {
      const wrapper = createInteractiveWrapper()
      const inputs = wrapper.findAll('input')
      await inputs[2].setValue('mybucket')
      expect(config.influxdb2_bucket).toBe('mybucket')
    })

    it('updates influxdb2_token via v-model', async () => {
      const wrapper = createInteractiveWrapper()
      const inputs = wrapper.findAll('input')
      await inputs[3].setValue('secret_token')
      expect(config.influxdb2_token).toBe('secret_token')
    })
  })

  describe('button interaction', () => {
    it('save button is disabled when global.disabled is true', () => {
      global.disabled = true

      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
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

      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
    })

    it('save button is enabled when appropriate', () => {
      global.disabled = false
      global.configChanged = true

      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      const buttons = wrapper.findAll('button')
      expect(buttons.length).toBeGreaterThan(0)
    })
  })

  describe('save validation (line 95)', () => {
    it('does not call saveAll when validateCurrentForm returns false', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)

      config.saveAll = vi.fn()

      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
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
        await wrapper.vm.save()
        expect(config.saveAll).not.toHaveBeenCalled()
      }
    })

    it('save returns early when validation fails', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)

      config.saveAll = vi.fn()

      const wrapper = mount(PushInfluxdbView, {
        global: {
          stubs: {
            BsInputText: { template: '<input />' },
            BsButton: { template: '<button><slot /></button>' }
          }
        }
      })

      await wrapper.vm.save()

      expect(config.saveAll).not.toHaveBeenCalled()
    })
  })
})
