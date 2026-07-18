import { beforeEach, vi, afterEach } from 'vitest'

// Set up import.meta.env variables for tests
if (!import.meta.env.VITE_APP_VERSION) {
  import.meta.env.VITE_APP_VERSION = '0.7.0'
}
if (!import.meta.env.VITE_APP_BUILD) {
  import.meta.env.VITE_APP_BUILD = 'test-build'
}

// Create spy wrappers for store methods using vi.hoisted
// This runs before module imports so spies are ready
const spies = vi.hoisted(() => ({
  // Config store spies
  configSendConfig: vi.fn().mockResolvedValue(true),
  configSaveAll: vi.fn().mockResolvedValue(true),
  configRestart: vi.fn().mockResolvedValue(true),
  configRunPushTest: vi.fn().mockResolvedValue(true),
  configLoad: vi.fn().mockResolvedValue(true),
  configRunWifiScan: vi.fn().mockResolvedValue(true),

  // Status store spies
  statusLoad: vi.fn().mockResolvedValue(true),

  // Global store spies
  globalClearMessages: vi.fn(),
  globalSetError: vi.fn(),
  globalSetSuccess: vi.fn()
}))

// Mock /modules/pinia to wrap real stores with spyable methods
vi.mock('@/modules/pinia', async () => {
  const actual = await vi.importActual('@/modules/pinia')

  // Wrap store methods with spies
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

// Mock localStorage FIRST before any other imports
const localStorageMock = {
  getItem: vi.fn(),
  setItem: vi.fn(),
  removeItem: vi.fn(),
  clear: vi.fn(),
  length: 0,
  key: vi.fn()
}

Object.defineProperty(window, 'localStorage', {
  value: localStorageMock
})

// Mock sessionStorage
const sessionStorageMock = {
  getItem: vi.fn(),
  setItem: vi.fn(),
  removeItem: vi.fn(),
  clear: vi.fn(),
  length: 0,
  key: vi.fn()
}

Object.defineProperty(window, 'sessionStorage', {
  value: sessionStorageMock
})

// Mock the espframework-ui-components library with proper spyable methods
const httpClientMocks = {
  request: vi.fn(),
  filesystemRequest: vi.fn(),
  getJson: vi.fn(),
  postJson: vi.fn(),
  get: vi.fn(),
  post: vi.fn(),
  uploadFile: vi.fn(),
  delete: vi.fn(),
  createWebSocket: vi.fn(() => ({
    open: vi.fn(),
    close: vi.fn(),
    send: vi.fn(),
    socketGetter: vi.fn(() => ({ readyState: 1 })),
    onOpen: null,
    onClose: null,
    onMessage: null,
    onError: null
  }))
}

vi.mock('@mp-se/espframework-ui-components', () => ({
  sharedHttpClient: httpClientMocks,
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn(),
  logWarn: vi.fn(),
  formatTime: vi.fn((seconds) => {
    const hours = Math.floor(seconds / 3600)
    const minutes = Math.floor((seconds % 3600) / 60)
    const secs = seconds % 60
    return `${hours}h ${minutes}m ${secs}s`
  }),
  tempToF: vi.fn((celsius) => (celsius * 9) / 5 + 32),
  tempToC: vi.fn((fahrenheit) => ((fahrenheit - 32) * 5) / 9),
  validateCurrentForm: vi.fn(() => true),
  BsInputText: { name: 'BsInputText', template: '<input />' },
  BsInputNumber: { name: 'BsInputNumber', template: '<input type="number" />' },
  BsInputSwitch: { name: 'BsInputSwitch', template: '<input type="checkbox" />' },
  BsInputRadio: { name: 'BsInputRadio', template: '<input type="radio" />' },
  BsInputReadonly: { name: 'BsInputReadonly', template: '<div></div>' },
  BsButton: { name: 'BsButton', template: '<button></button>' },
  BsMessage: { name: 'BsMessage', template: '<div><slot /></div>' },
  BsDropdown: { name: 'BsDropdown', template: '<select></select>' },
  BsSelect: { name: 'BsSelect', template: '<select></select>' },
  BsModal: { name: 'BsModal', template: '<div></div>' },
  BsCard: { name: 'BsCard', template: '<div></div>' },
  BsInputTextAreaFormat: { name: 'BsInputTextAreaFormat', template: '<textarea></textarea>' },
  BsFileUpload: { name: 'BsFileUpload', template: '<input type="file" />' },
  BsProgress: { name: 'BsProgress', template: '<div></div>' },
  BsModalConfirm: { name: 'BsModalConfirm', template: '<div></div>' }
}))

// Mock @vue/test-utils to auto-inject router context
vi.mock('@vue/test-utils', async () => {
  const actual = await vi.importActual('@vue/test-utils')

  const configureGlobalConfig = (component, options) => {
    const globalConfig = options.global || {}

    // Only add default mocks if no plugins are providing router
    const hasRouter = globalConfig.plugins?.some((p) => p.push && p.currentRoute)

    if (!hasRouter) {
      const mockRoute = {
        path: '/',
        name: 'test',
        params: {},
        query: {},
        fullPath: '/'
      }

      const mockRouter = {
        push: vi.fn(() => Promise.resolve()),
        replace: vi.fn(() => Promise.resolve()),
        go: vi.fn(),
        back: vi.fn(),
        currentRoute: { value: mockRoute }
      }

      const mocks = globalConfig.mocks || {}
      globalConfig.mocks = {
        ...mocks,
        $route: mockRoute,
        $router: mockRouter
      }

      globalConfig.provide = globalConfig.provide || {}
      globalConfig.provide.route = mockRoute
      globalConfig.provide.router = mockRouter
    }

    return globalConfig
  }

  const originalMount = actual.mount
  const originalShallowMount = actual.shallowMount

  // Wrap mount to automatically provide $route and $router
  const wrappedMount = (component, options = {}) => {
    const globalConfig = configureGlobalConfig(component, options)
    return originalMount(component, {
      ...options,
      global: globalConfig
    })
  }

  const wrappedShallowMount = (component, options = {}) => {
    const globalConfig = configureGlobalConfig(component, options)
    return originalShallowMount(component, {
      ...options,
      global: globalConfig
    })
  }

  return {
    ...actual,
    mount: wrappedMount,
    shallowMount: wrappedShallowMount
  }
})
vi.mock('vue-router', () => ({
  createRouter: vi.fn((options) => {
    // Use a mutable object to track route state
    let currentPath = '/'
    const mockRoute = {
      path: currentPath,
      name: 'test',
      params: {},
      query: {},
      fullPath: currentPath
    }

    const routerInstance = {
      install: vi.fn((app) => {
        // Add $route as a global property that components can access
        app.config.globalProperties.$route = mockRoute
        app.config.globalProperties.$router = {
          push: (to) => {
            currentPath = typeof to === 'string' ? to : to?.path || '/'
            mockRoute.path = currentPath
            mockRoute.fullPath = currentPath
            return Promise.resolve()
          },
          replace: vi.fn(() => Promise.resolve()),
          go: vi.fn(),
          back: vi.fn(),
          currentRoute: { value: mockRoute }
        }
      }),
      push: (to) => {
        currentPath = typeof to === 'string' ? to : to?.path || '/'
        mockRoute.path = currentPath
        mockRoute.fullPath = currentPath
        return Promise.resolve()
      },
      replace: vi.fn(() => Promise.resolve()),
      go: vi.fn(),
      back: vi.fn(),
      isReady: vi.fn(() => Promise.resolve()),
      beforeEach: vi.fn(),
      routes: options?.routes || [],
      currentRoute: {
        value: mockRoute
      }
    }

    return routerInstance
  }),
  createMemoryHistory: vi.fn(() => ({})),
  useRouter: vi.fn(() => ({
    push: vi.fn(() => Promise.resolve()),
    replace: vi.fn(() => Promise.resolve()),
    go: vi.fn(),
    back: vi.fn(),
    currentRoute: {
      value: {
        path: '/',
        name: 'test',
        params: {},
        query: {}
      }
    }
  })),
  useRoute: vi.fn(() => ({
    path: '/',
    name: 'test',
    params: {},
    query: {},
    fullPath: '/'
  }))
}))

// We removed the global globalStore mock because it was interfering with unit tests.
// Component tests can handle setting up their own store data when needed.
// Unit tests for globalStore should test the store in isolation with default values.

/**
 * Mock Pinia to automatically apply testing plugins to ALL Pinia instances
 * This ensures tests that create their own createPinia() get proper initialization
 */
vi.mock('pinia', async () => {
  const actual = await vi.importActual('pinia')

  return {
    ...actual
  }
})

// Reset mocks before each test and set up global router context
beforeEach(() => {
  // Reset HTTP client mocks
  httpClientMocks.request.mockReset()
  httpClientMocks.filesystemRequest.mockReset()
  httpClientMocks.getJson.mockReset()
  httpClientMocks.postJson.mockReset()
  httpClientMocks.get.mockReset()
  httpClientMocks.post.mockReset()
  httpClientMocks.uploadFile.mockReset()
  httpClientMocks.delete.mockReset()

  // Reset storage mocks
  localStorageMock.getItem.mockClear()
  localStorageMock.setItem.mockClear()
  localStorageMock.removeItem.mockClear()
  localStorageMock.clear.mockClear()
  sessionStorageMock.getItem.mockClear()
  sessionStorageMock.setItem.mockClear()
  sessionStorageMock.removeItem.mockClear()
  sessionStorageMock.clear.mockClear()

  // Reset spy functions
  spies.configSendConfig.mockClear()
  spies.configSaveAll.mockClear()
  spies.configRestart.mockClear()
  spies.configRunPushTest.mockClear()
  spies.configLoad.mockClear()
  spies.configRunWifiScan.mockClear()
  spies.statusLoad.mockClear()
  spies.globalClearMessages.mockClear()
  spies.globalSetError.mockClear()
  spies.globalSetSuccess.mockClear()
})

afterEach(() => {
  // Cleanup
  vi.clearAllMocks()
})
