import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import BrewgraphView from '../BrewgraphView.vue'
import { createPinia, setActivePinia } from 'pinia'
import { useGlobalStore } from '@/modules/globalStore'
import { useConfigStore } from '@/modules/configStore'

// Mock the pinia module
vi.mock('@/modules/pinia', () => ({
  get global() {
    return useGlobalStore()
  },
  get config() {
    return useConfigStore()
  }
}))

// Mock validation
vi.mock('@mp-se/espframework-ui-components', () => ({
  validateCurrentForm: vi.fn(() => true),
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn()
}))

const LOCAL_URL = 'http://192.168.1.10/ingest/chamber'

const mountView = () =>
  mount(BrewgraphView, {
    global: {
      stubs: {
        BsSelect: {
          template:
            '<select :data-label="label" :value="modelValue" :disabled="disabled" @change="$emit(\'update:modelValue\', Number($event.target.value))"><option v-for="o in options" :key="o.value" :value="o.value">{{ o.label }}</option></select>',
          props: ['modelValue', 'label', 'help', 'options', 'disabled']
        },
        BsInputText: {
          template:
            '<input :data-label="label" :type="type" :value="modelValue" :disabled="disabled" @input="$emit(\'update:modelValue\', $event.target.value)" />',
          props: ['modelValue', 'label', 'help', 'type', 'maxlength', 'disabled']
        },
        BsInputSwitch: {
          template:
            '<input type="checkbox" :data-label="label" :checked="modelValue" :disabled="disabled" @change="$emit(\'update:modelValue\', $event.target.checked)" />',
          props: ['modelValue', 'label', 'help', 'disabled']
        }
      }
    }
  })

describe('BrewgraphView', () => {
  let global, config
  beforeEach(async () => {
    const pinia = createPinia()
    setActivePinia(pinia)

    const stores = await import('@/modules/pinia')
    global = stores.global
    config = stores.config

    config.brewgraph_token = ''
    config.brewgraph_url = ''
    config.brewgraph_interval = 15
    config.brewgraph_remote_control_enabled = false
    config.saveAll = vi.fn().mockResolvedValue(true)
    global.disabled = false
    global.clearMessages = vi.fn()
    global.configChanged = false

    vi.clearAllMocks()
  })

  describe('component', () => {
    it('mounts and displays the page title', () => {
      const wrapper = mountView()
      expect(wrapper.exists()).toBe(true)
      expect(wrapper.text()).toContain('BrewGraph integration')
    })

    it('renders the form with a save button', () => {
      const wrapper = mountView()
      expect(wrapper.find('form.needs-validation').exists()).toBe(true)
      expect(wrapper.find('button[type="submit"]').text()).toContain('Save')
    })

    it('offers 5, 10, 15 and 20 minutes as the report interval, 15 by default', () => {
      const wrapper = mountView()
      const select = wrapper.find('select[data-label="Report interval"]')

      expect(select.exists()).toBe(true)
      expect(select.findAll('option').map((o) => o.text())).toEqual([
        '5 minutes',
        '10 minutes',
        '15 minutes',
        '20 minutes'
      ])
      expect(select.element.value).toBe('15')
    })

    it('renders the URL field as a url input and the token as a password input', () => {
      const wrapper = mountView()
      expect(wrapper.find('input[data-label="Server URL"]').attributes('type')).toBe('url')
      expect(wrapper.find('input[data-label="Device token"]').attributes('type')).toBe('password')
    })

    it('keeps BrewGraph control disabled by default', () => {
      const wrapper = mountView()
      const remote = wrapper.find('input[data-label="Allow BrewGraph to control the chamber"]')

      expect(remote.exists()).toBe(true)
      expect(remote.element.checked).toBe(false)
    })

    it('shows the stored values', () => {
      config.brewgraph_url = LOCAL_URL
      config.brewgraph_token = 'secret'
      const wrapper = mountView()

      expect(wrapper.find('input[data-label="Server URL"]').element.value).toBe(LOCAL_URL)
      expect(wrapper.find('input[data-label="Device token"]').element.value).toBe('secret')
    })
  })

  describe('v-model bindings', () => {
    it('updates brewgraph_url', async () => {
      const wrapper = mountView()
      await wrapper.find('input[data-label="Server URL"]').setValue(LOCAL_URL)
      expect(config.brewgraph_url).toBe(LOCAL_URL)
    })

    it('updates brewgraph_token', async () => {
      const wrapper = mountView()
      await wrapper.find('input[data-label="Device token"]').setValue('secret')
      expect(config.brewgraph_token).toBe('secret')
    })

    it('updates brewgraph_interval with a number', async () => {
      const wrapper = mountView()
      const select = wrapper.find('select[data-label="Report interval"]')
      select.element.value = '10'
      await select.trigger('change')
      expect(config.brewgraph_interval).toBe(10)
    })

    it('updates the BrewGraph control permission', async () => {
      const wrapper = mountView()
      await wrapper
        .find('input[data-label="Allow BrewGraph to control the chamber"]')
        .setValue(true)

      expect(config.brewgraph_remote_control_enabled).toBe(true)
    })

    it('disables the interval while the page is busy', () => {
      global.disabled = true
      const wrapper = mountView()
      expect(wrapper.find('select[data-label="Report interval"]').element.disabled).toBe(true)
    })

    it('disables both fields while the page is busy', () => {
      global.disabled = true
      const wrapper = mountView()

      expect(wrapper.find('input[data-label="Server URL"]').element.disabled).toBe(true)
      expect(wrapper.find('input[data-label="Device token"]').element.disabled).toBe(true)
    })
  })

  describe('save', () => {
    it('calls saveAll when the form is submitted', async () => {
      const wrapper = mountView()
      await wrapper.find('form').trigger('submit')
      await wrapper.vm.$nextTick()

      expect(config.saveAll).toHaveBeenCalledTimes(1)
    })

    it('does not call saveAll when validation fails', async () => {
      const { validateCurrentForm } = await import('@mp-se/espframework-ui-components')
      vi.mocked(validateCurrentForm).mockReturnValueOnce(false)

      const wrapper = mountView()
      await wrapper.vm.save()

      expect(config.saveAll).not.toHaveBeenCalled()
    })

    it('disables the save button when nothing changed or the page is busy', async () => {
      global.configChanged = false
      const wrapper = mountView()
      expect(wrapper.find('button[type="submit"]').element.disabled).toBe(true)

      global.configChanged = true
      await wrapper.vm.$nextTick()
      expect(wrapper.find('button[type="submit"]').element.disabled).toBe(false)

      global.disabled = true
      await wrapper.vm.$nextTick()
      expect(wrapper.find('button[type="submit"]').element.disabled).toBe(true)
    })
  })
})
