import { describe, it, expect, beforeEach, vi } from 'vitest'
import { createPinia, setActivePinia } from 'pinia'
import { sharedHttpClient as http } from '@mp-se/espframework-ui-components'

// Mock the HTTP client
vi.mock('@mp-se/espframework-ui-components', () => ({
  sharedHttpClient: {
    getJson: vi.fn()
  },
  logDebug: vi.fn(),
  logInfo: vi.fn(),
  logError: vi.fn(),
  logWarn: vi.fn()
}))

import { useGlobalStore } from '../globalStore'

describe('globalStore', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
    vi.clearAllMocks()
  })

  describe('state initialization', () => {
    it('initializes with default values', () => {
      const store = useGlobalStore()
      expect(store.id).toBe('')
      expect(store.platform).toBe('')
      expect(store.board).toBe('')
      expect(store.app_ver).toBe('')
      expect(store.app_build).toBe('')
      expect(store.initialized).toBe(false)
      expect(store.disabled).toBe(false)
      expect(store.configChanged).toBe(false)
    })

    it('initializes ui configuration', () => {
      const store = useGlobalStore()
      expect(store.ui).toBeDefined()
      expect(store.ui.enableVoltageFragment).toBe(false)
      expect(store.ui.enableManualWifiEntry).toBe(false)
      expect(store.ui.enableScanForStrongestAp).toBe(false)
    })

    it('initializes feature flags', () => {
      const store = useGlobalStore()
      expect(store.feature).toBeDefined()
      expect(store.feature.ble).toBe(true)
      expect(store.feature.ble_sensor).toBe(true)
    })

    it('initializes messages as empty strings', () => {
      const store = useGlobalStore()
      expect(store.messageError).toBe('')
      expect(store.messageWarning).toBe('')
      expect(store.messageSuccess).toBe('')
      expect(store.messageInfo).toBe('')
    })
  })

  describe('getters', () => {
    it('isError returns true when messageError is set', () => {
      const store = useGlobalStore()
      store.messageError = 'Something went wrong'
      expect(store.isError).toBe(true)
    })

    it('isError returns false when messageError is empty', () => {
      const store = useGlobalStore()
      store.messageError = ''
      expect(store.isError).toBe(false)
    })

    it('isWarning returns true when messageWarning is set', () => {
      const store = useGlobalStore()
      store.messageWarning = 'Something needs attention'
      expect(store.isWarning).toBe(true)
    })

    it('isWarning returns false when messageWarning is empty', () => {
      const store = useGlobalStore()
      store.messageWarning = ''
      expect(store.isWarning).toBe(false)
    })

    it('isSuccess returns true when messageSuccess is set', () => {
      const store = useGlobalStore()
      store.messageSuccess = 'Operation completed'
      expect(store.isSuccess).toBe(true)
    })

    it('isSuccess returns false when messageSuccess is empty', () => {
      const store = useGlobalStore()
      store.messageSuccess = ''
      expect(store.isSuccess).toBe(false)
    })

    it('isInfo returns true when messageInfo is set', () => {
      const store = useGlobalStore()
      store.messageInfo = 'Information message'
      expect(store.isInfo).toBe(true)
    })

    it('isInfo returns false when messageInfo is empty', () => {
      const store = useGlobalStore()
      store.messageInfo = ''
      expect(store.isInfo).toBe(false)
    })
  })

  describe('clearMessages action', () => {
    it('clears all message types', () => {
      const store = useGlobalStore()
      store.messageError = 'Error'
      store.messageWarning = 'Warning'
      store.messageSuccess = 'Success'
      store.messageInfo = 'Info'

      store.clearMessages()

      expect(store.messageError).toBe('')
      expect(store.messageWarning).toBe('')
      expect(store.messageSuccess).toBe('')
      expect(store.messageInfo).toBe('')
    })

    it('clears messages when none are set', () => {
      const store = useGlobalStore()
      store.clearMessages()

      expect(store.messageError).toBe('')
      expect(store.messageWarning).toBe('')
      expect(store.messageSuccess).toBe('')
      expect(store.messageInfo).toBe('')
    })
  })

  describe('load action', () => {
    it('fetches feature data from API', async () => {
      const mockData = {
        board: 'ESP32',
        app_ver: '3.0.0',
        app_build: '12345',
        platform: 'ESPRESSIF',
        firmware_file: 'firmware.bin',
        ble: true,
        ble_sensor: true
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useGlobalStore()
      const result = await store.load()

      expect(result).toBe(true)
      expect(http.getJson).toHaveBeenCalledWith('api/feature')
    })

    it('updates store with API response data', async () => {
      const mockData = {
        board: 'ESP32S3',
        app_ver: '3.1.0',
        app_build: '54321',
        platform: 'ESPRESSIF32',
        firmware_file: 'firmware.bin',
        ble: false,
        ble_sensor: false
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useGlobalStore()
      await store.load()

      expect(store.board).toBe('ESP32S3')
      expect(store.app_ver).toBe('3.1.0')
      expect(store.app_build).toBe('54321')
      expect(store.platform).toBe('ESPRESSIF32')
      expect(store.firmware_file).toBe('firmware.bin')
      expect(store.feature.ble).toBe(false)
      expect(store.feature.ble_sensor).toBe(false)
    })

    it('converts board to uppercase', async () => {
      const mockData = {
        board: 'esp32',
        app_ver: '3.0.0',
        app_build: '12345',
        platform: 'espressif',
        firmware_file: 'firmware.bin',
        ble: true,
        ble_sensor: true
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useGlobalStore()
      await store.load()

      expect(store.board).toBe('ESP32')
      expect(store.platform).toBe('ESPRESSIF')
    })

    it('converts firmware_file to lowercase', async () => {
      const mockData = {
        board: 'ESP32',
        app_ver: '3.0.0',
        app_build: '12345',
        platform: 'ESPRESSIF',
        firmware_file: 'FIRMWARE.BIN',
        ble: true,
        ble_sensor: true
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useGlobalStore()
      await store.load()

      expect(store.firmware_file).toBe('firmware.bin')
    })

    it('handles API error gracefully', async () => {
      http.getJson.mockRejectedValue(new Error('Network error'))

      const store = useGlobalStore()
      const result = await store.load()

      expect(result).toBe(false)
    })

    it('returns false on failure', async () => {
      const error = new Error('Failed to fetch features')
      http.getJson.mockRejectedValue(error)

      const store = useGlobalStore()
      const result = await store.load()

      expect(result).toBe(false)
    })

    it('preserves existing state on error', async () => {
      const store = useGlobalStore()
      store.board = 'ESP32'
      store.app_ver = '2.9.0'

      http.getJson.mockRejectedValue(new Error('API failed'))

      await store.load()

      // State should be preserved on error
      expect(store.board).toBe('ESP32')
      expect(store.app_ver).toBe('2.9.0')
    })

    it('handles missing ble field in response', async () => {
      const mockData = {
        board: 'ESP32',
        app_ver: '3.0.0',
        app_build: '12345',
        platform: 'ESPRESSIF',
        firmware_file: 'firmware.bin'
        // ble and ble_sensor are missing
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useGlobalStore()
      await store.load()

      // Should maintain previous values if not in response
      expect(store.feature.ble).toBeDefined()
      expect(store.feature.ble_sensor).toBeDefined()
    })
  })

  describe('store state mutations', () => {
    it('allows setting messageError', () => {
      const store = useGlobalStore()
      store.messageError = 'Test error'
      expect(store.messageError).toBe('Test error')
    })

    it('allows setting disabled flag', () => {
      const store = useGlobalStore()
      store.disabled = true
      expect(store.disabled).toBe(true)
    })

    it('allows setting configChanged flag', () => {
      const store = useGlobalStore()
      store.configChanged = true
      expect(store.configChanged).toBe(true)
    })

    it('allows updating feature flags', () => {
      const store = useGlobalStore()
      store.feature.ble = false
      expect(store.feature.ble).toBe(false)
    })

    it('allows updating ui configuration', () => {
      const store = useGlobalStore()
      store.ui.enableVoltageFragment = true
      expect(store.ui.enableVoltageFragment).toBe(true)
    })
  })
})
