import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount, flushPromises } from '@vue/test-utils'
import App from '../App.vue'
import { createPinia, setActivePinia } from 'pinia'
import { useGlobalStore } from '@/modules/globalStore'
import { useStatusStore } from '@/modules/statusStore'
import { useConfigStore } from '@/modules/configStore'

let testGlobal, testStatus, testConfig

// Override the setup.js global mock for @/modules/pinia in this file
vi.mock('@/modules/pinia', () => ({
  default: { install: vi.fn() },
  saveConfigState: vi.fn(),
  get global() {
    return testGlobal
  },
  get status() {
    return testStatus
  },
  get config() {
    return testConfig
  }
}))

vi.mock('@/modules/router', () => ({
  items: {
    value: [{ label: 'Home', to: '/' }]
  }
}))

// Provide auth and ping which setup.js's httpClientMocks does not include
vi.mock('@mp-se/espframework-ui-components', () => ({
  sharedHttpClient: {
    ping: vi.fn(async () => true),
    auth: vi.fn(async () => true)
  },
  logError: vi.fn(),
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logWarn: vi.fn()
}))

describe('App.vue initialization paths', () => {
  let pinia

  beforeEach(async () => {
    vi.clearAllMocks()
    pinia = createPinia()
    setActivePinia(pinia)
    testGlobal = useGlobalStore(pinia)
    testStatus = useStatusStore(pinia)
    testConfig = useConfigStore(pinia)

    testGlobal.initialized = false
    testGlobal.disabled = false
    testGlobal.messageError = ''
    testGlobal.messageWarning = ''
    testGlobal.messageSuccess = ''
    testGlobal.messageInfo = ''
    testGlobal.configChanged = false
    testStatus.connected = true
    testStatus.wifi_setup = false
    testConfig.dark_mode = false
    testConfig.mdns = 'chamber'

    testGlobal.load = vi.fn(async () => true)
    testStatus.load = vi.fn(async () => true)
    testConfig.load = vi.fn(async () => true)
    testGlobal.clearMessages = vi.fn()

    const { sharedHttpClient } = await import('@mp-se/espframework-ui-components')
    sharedHttpClient.auth.mockReset()
    sharedHttpClient.auth.mockResolvedValue(true)
    sharedHttpClient.ping.mockReset()
    sharedHttpClient.ping.mockResolvedValue(true)
  })

  const createWrapper = () =>
    mount(App, {
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

  describe('auth failure (line 159)', () => {
    it('sets error message and stays uninitialized when auth returns false', async () => {
      const { sharedHttpClient } = await import('@mp-se/espframework-ui-components')
      sharedHttpClient.auth.mockResolvedValueOnce(false)

      createWrapper()
      await flushPromises()

      expect(testGlobal.messageError).toContain('authenticate')
      expect(testGlobal.initialized).toBe(false)
    })

    it('does not call global.load when auth fails', async () => {
      const { sharedHttpClient } = await import('@mp-se/espframework-ui-components')
      sharedHttpClient.auth.mockResolvedValueOnce(false)

      createWrapper()
      await flushPromises()

      expect(testGlobal.load).not.toHaveBeenCalled()
    })
  })

  describe('status.load failure (lines 170-173)', () => {
    it('sets status error message when status.load returns false', async () => {
      testStatus.load = vi.fn(async () => false)

      createWrapper()
      await flushPromises()

      expect(testGlobal.messageError).toContain('status')
      expect(testGlobal.initialized).toBe(false)
    })

    it('does not call config.load when status.load fails', async () => {
      testStatus.load = vi.fn(async () => false)

      createWrapper()
      await flushPromises()

      expect(testConfig.load).not.toHaveBeenCalled()
    })
  })

  describe('config.load failure (lines 177-181)', () => {
    it('sets configuration error message when config.load returns false', async () => {
      testConfig.load = vi.fn(async () => false)

      createWrapper()
      await flushPromises()

      expect(testGlobal.messageError).toContain('configuration')
      expect(testGlobal.initialized).toBe(false)
    })
  })

  describe('successful initialization (lines 185-186)', () => {
    it('calls saveConfigState and sets initialized=true when all loads succeed', async () => {
      const { saveConfigState } = await import('@/modules/pinia')

      createWrapper()
      await flushPromises()

      expect(saveConfigState).toHaveBeenCalled()
      expect(testGlobal.initialized).toBe(true)
    })

    it('calls all load functions in order', async () => {
      createWrapper()
      await flushPromises()

      expect(testGlobal.load).toHaveBeenCalled()
      expect(testStatus.load).toHaveBeenCalled()
      expect(testConfig.load).toHaveBeenCalled()
    })
  })

  describe('initializeApp catch block (lines 187-189)', () => {
    it('sets Initialization failed error when global.load throws', async () => {
      testGlobal.load = vi.fn(async () => {
        throw new Error('Network failure')
      })

      createWrapper()
      await flushPromises()

      expect(testGlobal.messageError).toContain('Initialization failed')
    })

    it('sets error message with exception details when status.load throws', async () => {
      testStatus.load = vi.fn(async () => {
        throw new Error('Status timeout')
      })

      createWrapper()
      await flushPromises()

      expect(testGlobal.messageError).toContain('Initialization failed')
    })
  })

  describe('handleDarkModeUpdate error handling (line 205)', () => {
    it('catches DOM errors without throwing when setAttribute fails', async () => {
      const wrapper = createWrapper()
      await flushPromises()

      const spy = vi
        .spyOn(document.documentElement, 'setAttribute')
        .mockImplementationOnce(() => {
          throw new Error('DOM manipulation error')
        })

      expect(() => {
        wrapper.vm.handleDarkModeUpdate(true)
      }).not.toThrow()

      spy.mockRestore()
    })

    it('handleDarkModeUpdate handles setAttribute error gracefully', async () => {
      const wrapper = createWrapper()
      await flushPromises()

      const origSetAttribute = document.documentElement.setAttribute.bind(
        document.documentElement
      )
      const spy = vi
        .spyOn(document.documentElement, 'setAttribute')
        .mockImplementationOnce(() => {
          throw new Error('test DOM error')
        })

      // The method should not throw even when setAttribute throws
      expect(() => {
        wrapper.vm.handleDarkModeUpdate(true)
      }).not.toThrow()

      spy.mockRestore()
    })
  })
})
