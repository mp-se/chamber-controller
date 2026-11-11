import { vi } from 'vitest'
import { createI18n } from 'vue-i18n'
import { readFileSync } from 'node:fs'
import { resolve, dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

// Load translations
const __dirname = dirname(fileURLToPath(import.meta.url))
const en = JSON.parse(readFileSync(resolve(__dirname, '../locales/en.json'), 'utf-8'))

const i18n = createI18n({
  legacy: false,
  locale: 'en',
  fallbackLocale: 'en',
  messages: { en }
})

// Set up import.meta.env variables for tests
if (!import.meta.env.VITE_APP_VERSION) {
  import.meta.env.VITE_APP_VERSION = '0.7.0'
}
if (!import.meta.env.VITE_APP_BUILD) {
  import.meta.env.VITE_APP_BUILD = 'test-build'
}

// Create spy wrappers for store methods
const spies = vi.hoisted(() => ({
  configSendConfig: vi.fn().mockResolvedValue(true),
  configSaveAll: vi.fn().mockResolvedValue(true),
  configRestart: vi.fn().mockResolvedValue(true),
  configRunPushTest: vi.fn().mockResolvedValue(true),
  configLoad: vi.fn().mockResolvedValue(true),
  configRunWifiScan: vi.fn().mockResolvedValue(true),
  statusLoad: vi.fn().mockResolvedValue(true),
  globalClearMessages: vi.fn(),
  globalSetError: vi.fn(),
  globalSetSuccess: vi.fn()
}))

// Mock /modules/pinia
vi.mock('@/modules/pinia', async () => {
  const actual = await vi.importActual('@/modules/pinia')
  actual.config.sendConfig = spies.configSendConfig
  actual.config.saveAll = spies.configSaveAll
  actual.config.restart = spies.configRestart
  actual.config.runPushTest = spies.configRunPushTest
  actual.config.load = spies.configLoad
  actual.config.runWifiScan = spies.configRunWifiScan
  actual.status.load = spies.statusLoad
  actual.global.clearMessages = spies.globalClearMessages
  actual.global.setError = spies.globalSetError
  actual.global.setSuccess = spies.globalSetSuccess
  return actual
})

// Mock vue-i18n to use the real instance in setup()
vi.mock('vue-i18n', async () => {
  const actual = await vi.importActual('vue-i18n')
  return {
    ...actual,
    useI18n: () => ({
      t: i18n.global.t,
      tc: i18n.global.tc,
      te: i18n.global.te,
      d: i18n.global.d,
      n: i18n.global.n,
      locale: i18n.global.locale
    })
  }
})

// Mock localStorage
const localStorageMock = {
  getItem: vi.fn(), setItem: vi.fn(), removeItem: vi.fn(),
  clear: vi.fn(), length: 0, key: vi.fn()
}
Object.defineProperty(window, 'localStorage', { value: localStorageMock })
Object.defineProperty(window, 'confirm', { value: vi.fn(() => true) })
Object.defineProperty(window, 'alert', { value: vi.fn() })

// Mock components
const componentMocks = {
  BsInputText: { name: 'BsInputText', template: '<input />' },
  BsInputNumber: { name: 'BsInputNumber', template: '<input type="number" />' },
  BsInputSwitch: { name: 'BsInputSwitch', template: '<input type="checkbox" />' },
  BsInputRadio: { name: 'BsInputRadio', template: '<div class="bs-input-radio"><input type="radio" /></div>' },
  BsInputReadonly: { name: 'BsInputReadonly', template: '<div></div>' },
  BsButton: { name: 'BsButton', template: '<button><slot /></button>' },
  BsMessage: { name: 'BsMessage', template: '<div><slot /></div>' },
  BsDropdown: { name: 'BsDropdown', template: '<select><slot /></select>' },
  BsSelect: { name: 'BsSelect', template: '<select><slot /></select>' },
  BsModal: { name: 'BsModal', template: '<div><slot /></div>' },
  BsCard: { name: 'BsCard', template: '<div><slot /></div>' },
  BsInputTextAreaFormat: { name: 'BsInputTextAreaFormat', template: '<textarea></textarea>' },
  BsFileUpload: { name: 'BsFileUpload', template: '<input type="file" />' },
  BsProgress: { name: 'BsProgress', template: '<div></div>' },
  BsModalConfirm: { name: 'BsModalConfirm', template: '<div></div>' }
}

const httpClientMocks = {
  request: vi.fn(), filesystemRequest: vi.fn(), getJson: vi.fn(), postJson: vi.fn(),
  get: vi.fn(), post: vi.fn(), uploadFile: vi.fn(), delete: vi.fn(),
  createWebSocket: vi.fn(() => ({
    open: vi.fn(), close: vi.fn(), send: vi.fn(), socketGetter: vi.fn(() => ({ readyState: 1 })),
    onOpen: null, onClose: null, onMessage: null, onError: null
  }))
}

vi.mock('@mp-se/espframework-ui-components', () => ({
  sharedHttpClient: httpClientMocks,
  logDebug: vi.fn(), logInfo: vi.fn(), logError: vi.fn(), logWarn: vi.fn(),
  formatTime: vi.fn((s) => s),
  tempToF: vi.fn((c) => (c * 9) / 5 + 32),
  tempToC: vi.fn((f) => ((f - 32) * 5) / 9),
  validateCurrentForm: vi.fn(() => true),
  ...componentMocks
}))

// Mock @vue/test-utils
vi.mock('@vue/test-utils', async () => {
  const actual = await vi.importActual('@vue/test-utils')
  const originalMount = actual.mount
  const originalShallowMount = actual.shallowMount

  const configureGlobalConfig = (options) => {
    const globalConfig = options.global || {}
    globalConfig.plugins = globalConfig.plugins || []
    if (!globalConfig.plugins.includes(i18n)) {
      globalConfig.plugins.push(i18n)
    }

    const hasRouter = globalConfig.plugins.some((p) => p.push && p.currentRoute)
    if (!hasRouter) {
      const mockRoute = { path: '/', name: 'test', params: {}, query: {}, fullPath: '/' }
      const mockRouter = {
        push: vi.fn(() => Promise.resolve()), replace: vi.fn(() => Promise.resolve()),
        go: vi.fn(), back: vi.fn(), currentRoute: { value: mockRoute }
      }
      globalConfig.mocks = { ...globalConfig.mocks, $route: mockRoute, $router: mockRouter }
    }

    globalConfig.components = { ...globalConfig.components, ...componentMocks }
    return globalConfig
  }

  return {
    ...actual,
    mount: (c, o = {}) => originalMount(c, { ...o, global: configureGlobalConfig(o) }),
    shallowMount: (c, o = {}) => originalShallowMount(c, { ...o, global: configureGlobalConfig(o) })
  }
})
