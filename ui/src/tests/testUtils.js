import { createPinia, setActivePinia } from 'pinia'
import { vi } from 'vitest'

/**
 * Create a fresh Pinia instance and set it as active for tests
 * Automatically applies the testing plugin that mocks store actions
 * @returns {Object} Pinia instance
 */
export function createTestingPinia() {
  const pinia = createPinia()

  // First, apply plugin to intercept ALL store creation
  pinia.use(({ options, store }) => {
    // Get store ID (works for both old and new Pinia API)
    const storeId = store.$id || options.id

    // Mock config store actions
    const configIds = ['configStore', 'config']
    if (configIds.includes(storeId)) {
      store.sendConfig = vi.fn().mockResolvedValue(true)
      store.saveAll = vi.fn().mockResolvedValue(true)
      store.restart = vi.fn().mockResolvedValue(true)
      store.runPushTest = vi.fn().mockResolvedValue(true)
      store.load = vi.fn().mockResolvedValue(true)
      store.runWifiScan = vi.fn().mockResolvedValue(true)
    }

    // Mock status store actions
    const statusIds = ['statusStore', 'status']
    if (statusIds.includes(storeId)) {
      store.load = vi.fn().mockResolvedValue(true)
    }

    // Mock global store methods
    const globalIds = ['globalStore', 'global']
    if (globalIds.includes(storeId)) {
      store.clearMessages = vi.fn()
      store.setError = vi.fn()
      store.setSuccess = vi.fn()
    }
  })

  setActivePinia(pinia)
  return pinia
}

/**
 * Setup test environment with pinia and pre-mocked store actions
 * This must be called after importing stores to intercept their methods
 * @returns {Object} { pinia, config, global, status }
 */
export async function setupTestingEnvironmentWithMocks() {
  const pinia = createTestingPinia()

  // Import stores AFTER creating and setting the pinia instance
  const { config, global: globalStore, status } = await import('@/modules/pinia')

  // Mock all async store methods as vi.fn() so they can be spied on
  config.sendConfig = vi.fn().mockResolvedValue(true)
  config.saveAll = vi.fn().mockResolvedValue(true)
  config.restart = vi.fn().mockResolvedValue(true)
  config.runPushTest = vi.fn().mockResolvedValue(true)
  config.load = vi.fn().mockResolvedValue(true)
  config.runWifiScan = vi.fn().mockResolvedValue(true)

  // Mock status methods
  status.load = vi.fn().mockResolvedValue(true)

  // Mock global store methods
  globalStore.clearMessages = vi.fn()
  globalStore.setError = vi.fn()
  globalStore.setSuccess = vi.fn()

  return { pinia, config, global: globalStore, status }
}

/**
 * Reset all Vitest mocks
 */
export function resetAllMocks() {
  vi.clearAllMocks()
}

/**
 * Mock Pinia stores with default state
 * Provides a reusable store mock configuration
 */
export const mockConfigState = {
  id: 'test-device-id',
  mdns: 'test-device',
  token: 'test-token',
  host: '192.168.1.100',
  port: 80,
  secure: false,
  apiVersion: '3.0'
}

export const mockGlobalState = {
  disabled: false,
  configChanged: false,
  messageError: '',
  messageSuccess: '',
  clearMessages: vi.fn(),
  setError: vi.fn(),
  setSuccess: vi.fn()
}

export const mockStatusState = {
  isConnected: true,
  lastUpdate: new Date().toISOString(),
  status: {},
  clearStatus: vi.fn()
}

/**
 * Common component stubs for Vue Test Utils
 * Use these in mount() global.stubs config
 */
export const commonStubs = {
  BsButton: true,
  BsMessage: {
    template: '<div><slot /></div>'
  },
  BsInputText: true,
  BsInputNumber: true,
  BsInputSwitch: true,
  BsDropdown: true,
  BsModal: true,
  BsInputTextAreaFormat: true,
  RouterLink: true,
  RouterView: true
}

/**
 * Setup test environment before each test
 * Call this in beforeEach() to reset stores and mocks
 */
export function setupTestEnvironment() {
  const pinia = createTestingPinia()
  resetAllMocks()
  return { pinia }
}

/**
 * Create additional stubs for child components not in commonStubs
 */
export const additionalStubs = {
  VoltageFragment: true,
  ListFilesFragment: true,
  AdvancedFilesFragment: true,
  EnableCorsFragment: true,
  PidTemperatureFragment: true,
  PidControllerFragment: true,
  PidDataFragment: true,
  BsInputReadonly: true,
  BsInputTextAreaFormat: true,
  BsCard: true,
  BsFileUpload: true,
  BsProgress: true,
  BsModalConfirm: true
}

/**
 * Extended stubs combining common and additional
 */
export const allStubs = {
  ...commonStubs,
  ...additionalStubs
}
