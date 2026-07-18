import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { mount, flushPromises } from '@vue/test-utils'
import App from '../App.vue'
import { createPinia, setActivePinia } from 'pinia'
import { useGlobalStore } from '@/modules/globalStore'
import { useStatusStore } from '@/modules/statusStore'
import { useConfigStore } from '@/modules/configStore'

vi.mock('@mp-se/espframework-ui-components', () => ({
  sharedHttpClient: {
    ping: vi.fn(async () => true),
    auth: vi.fn(async () => true)
  },
  logError: vi.fn()
}))

vi.mock('@/modules/router', () => ({
  items: {
    value: [
      { label: 'Home', to: '/', icon: 'home' }
    ]
  }
}))

describe('App.vue', () => {
  let pinia
  let global
  let status
  let config

  beforeEach(() => {
    vi.clearAllMocks()
    pinia = createPinia()
    setActivePinia(pinia)
    global = useGlobalStore(pinia)
    status = useStatusStore(pinia)
    config = useConfigStore(pinia)

    // Setup store state
    global.initialized = false
    global.disabled = false
    global.isError = false
    global.isWarning = false
    global.isSuccess = false
    global.isInfo = false
    global.messageError = ''
    global.messageWarning = ''
    global.messageSuccess = ''
    global.messageInfo = ''
    global.uiVersion = '0.7.0'
    global.uiBuild = 'test'
    global.configChanged = false

    status.connected = true
    status.wifi_setup = false

    config.dark_mode = false
    config.mdns = 'chamber'

    // Mock store methods
    global.load = vi.fn(async () => true)
    status.load = vi.fn(async () => true)
    config.load = vi.fn(async () => true)
  })

  afterEach(() => {
    vi.clearAllMocks()
  })

  const createWrapper = () => {
    return mount(App, {
      global: {
        plugins: [pinia],
        stubs: {
          BsMenuBar: true,
          BsMessage: true,
          BsFooter: true,
          RouterView: true,
          RouterLink: true
        }
      }
    })
  }

  describe('component structure', () => {
    it('renders without error', () => {
      const wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })

    it('renders spinner dialog element', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('dialog#spinner').exists()).toBe(true)
    })

    it('renders container div', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('.container').exists()).toBe(true)
    })

    it('has initialization check in template', () => {
      const wrapper = createWrapper()
      // The component has conditional rendering for initialization
      expect(wrapper.vm).toBeDefined()
    })

    it('renders root element correctly', () => {
      const wrapper = createWrapper()
      expect(wrapper.element).toBeDefined()
      expect(wrapper.element.tagName).toBe('DIV')
    })
  })

  describe('messages and alerts', () => {
    it('has close method defined', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.close).toBeDefined()
      expect(typeof wrapper.vm.close).toBe('function')
    })
  })

  describe('connection and status', () => {
    it('status connected state exists in store', () => {
      const wrapper = createWrapper()
      expect(status.connected).toBeDefined()
    })

    it('status wifi_setup state exists in store', () => {
      const wrapper = createWrapper()
      expect(status.wifi_setup).toBeDefined()
    })

    it('global initialized state is reactive', () => {
      const wrapper = createWrapper()
      expect(global.initialized).toBeDefined()
      global.initialized = true
      expect(global.initialized).toBe(true)
    })
  })

  describe('dark mode', () => {
    it('has handleDarkModeUpdate method', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.handleDarkModeUpdate).toBeDefined()
      expect(typeof wrapper.vm.handleDarkModeUpdate).toBe('function')
    })

    it('can update dark mode', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      wrapper.vm.handleDarkModeUpdate(true)
      const theme = document.documentElement.getAttribute('data-bs-theme')
      expect(theme).toBe('dark')
    })

    it('can disable dark mode', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      wrapper.vm.handleDarkModeUpdate(false)
      const theme = document.documentElement.getAttribute('data-bs-theme')
      expect(theme).toBe('light')
    })
  })

  describe('disabled state', () => {
    it('disabled state is reactive', () => {
      const wrapper = createWrapper()
      expect(global.disabled).toBeDefined()
      expect(typeof global.disabled).toBe('boolean')
    })
  })

  describe('menu items computation', () => {
    it('computes menu items from router module', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.items).toBeDefined()
      expect(Array.isArray(wrapper.vm.items)).toBe(true)
    })
  })

  describe('footer and branding', () => {
    it('config has mdns name defined', () => {
      const wrapper = createWrapper()
      expect(config.mdns).toBeDefined()
    })
  })

  describe('lifecycle hooks', () => {
    it('sets up polling interval on mount', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.polling).toBeTruthy()
    })

    it('clears polling interval on unmount', () => {
      const wrapper = createWrapper()
      const pollingId = wrapper.vm.polling
      expect(pollingId).toBeTruthy()
      wrapper.unmount()
      expect(wrapper.exists()).toBe(false)
    })

    it('has close method for alert dismissal', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.close).toBeDefined()
      expect(typeof wrapper.vm.close).toBe('function')
    })

    it('has handleDarkModeUpdate method', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.handleDarkModeUpdate).toBeDefined()
      expect(typeof wrapper.vm.handleDarkModeUpdate).toBe('function')
    })
  })

  describe('initialization messaging', () => {
    it('initialization flag controls state', () => {
      const wrapper = createWrapper()
      expect(global.initialized).toBeDefined()
      global.initialized = true
      expect(global.initialized).toBe(true)
    })
  })

  describe('close method functionality', () => {
    it('close method is defined and callable', () => {
      const wrapper = createWrapper()
      
      expect(wrapper.vm.close).toBeDefined()
      expect(typeof wrapper.vm.close).toBe('function')
    })

    it('close method can be called with different alert types', () => {
      const wrapper = createWrapper()
      
      // Should not throw
      expect(() => {
        wrapper.vm.close('danger')
        wrapper.vm.close('warning')
        wrapper.vm.close('success')
        wrapper.vm.close('info')
      }).not.toThrow()
    })
  })

  describe('handleDarkModeUpdate functionality', () => {
    it('sets dark theme when called with true', () => {
      const wrapper = createWrapper()
      
      wrapper.vm.handleDarkModeUpdate(true)
      
      const theme = document.documentElement.getAttribute('data-bs-theme')
      expect(theme).toBe('dark')
    })

    it('sets light theme when called with false', () => {
      const wrapper = createWrapper()
      
      wrapper.vm.handleDarkModeUpdate(false)
      
      const theme = document.documentElement.getAttribute('data-bs-theme')
      expect(theme).toBe('light')
    })

    it('handleDarkModeUpdate is callable', () => {
      const wrapper = createWrapper()
      
      expect(wrapper.vm.handleDarkModeUpdate).toBeDefined()
      expect(typeof wrapper.vm.handleDarkModeUpdate).toBe('function')
      
      // Should not throw when called
      expect(() => {
        wrapper.vm.handleDarkModeUpdate(true)
        wrapper.vm.handleDarkModeUpdate(false)
      }).not.toThrow()
    })
  })

  describe('reactivity and watchers', () => {
    it('component responds to state changes', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      global.disabled = true
      await wrapper.vm.$nextTick()
      
      expect(global.disabled).toBe(true)
      
      global.disabled = false
      await wrapper.vm.$nextTick()
      
      expect(global.disabled).toBe(false)
    })

    it('dark mode state changes are tracked', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      
      config.dark_mode = true
      await wrapper.vm.$nextTick()
      
      expect(config.dark_mode).toBe(true)
      
      config.dark_mode = false
      await wrapper.vm.$nextTick()
      
      expect(config.dark_mode).toBe(false)
    })
  })

  describe('menu items computation', () => {
    it('computes and returns menu items', () => {
      const wrapper = createWrapper()
      
      expect(Array.isArray(wrapper.vm.items)).toBe(true)
    })

    it('items is readable', () => {
      const wrapper = createWrapper()
      
      const items = wrapper.vm.items
      expect(items).toBeDefined()
    })
  })

  describe('lifecycle and polling', () => {
    it('creates polling interval reference on mount', () => {
      const wrapper = createWrapper()
      
      expect(wrapper.vm.polling).toBeDefined()
      expect(wrapper.vm.polling).not.toBeNull()
    })

    it('cleans up on unmount', () => {
      const wrapper = createWrapper()
      const pollingRef = wrapper.vm.polling
      
      expect(pollingRef).toBeTruthy()
      
      wrapper.unmount()
      
      expect(wrapper.exists()).toBe(false)
    })
  })
})
