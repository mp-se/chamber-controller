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

import { useStatusStore } from '../statusStore'

describe('statusStore', () => {
  beforeEach(() => {
    setActivePinia(createPinia())
    vi.clearAllMocks()
  })

  describe('state initialization', () => {
    it('initializes with default values', () => {
      const store = useStatusStore()
      expect(store.id).toBe('')
      expect(store.rssi).toBe(0)
      expect(store.mdns).toBe('')
      expect(store.wifi_ssid).toBe('')
      expect(store.ip).toBe('')
      expect(store.connected).toBe(true)
    })

    it('initializes PID state', () => {
      const store = useStatusStore()
      expect(store.pid_mode).toBe('')
      expect(store.pid_state).toBe(0)
      expect(store.pid_state_string).toBe('')
      expect(store.pid_beer_temp).toBe(0)
      expect(store.pid_fridge_temp).toBe(0)
    })

    it('initializes uptime fields', () => {
      const store = useStatusStore()
      expect(store.uptime_seconds).toBe(0)
      expect(store.uptime_minutes).toBe(0)
      expect(store.uptime_hours).toBe(0)
      expect(store.uptime_days).toBe(0)
    })

    it('initializes temperature_device as empty array', () => {
      const store = useStatusStore()
      expect(Array.isArray(store.temperature_device)).toBe(true)
      expect(store.temperature_device.length).toBe(0)
    })

    it('initializes actuator states to false', () => {
      const store = useStatusStore()
      expect(store.pid_cooling_actuator_active).toBe(false)
      expect(store.pid_heating_actuator_active).toBe(false)
    })

    it('initializes sensor connection flags to false', () => {
      const store = useStatusStore()
      expect(store.pid_beer_temp_connected).toBe(false)
      expect(store.pid_fridge_temp_connected).toBe(false)
    })

    it('initializes heap memory fields', () => {
      const store = useStatusStore()
      expect(store.total_heap).toBe(0)
      expect(store.free_heap).toBe(0)
    })
  })

  describe('load action', () => {
    it('fetches status from API', async () => {
      const mockData = {
        id: 'brewlogger-01',
        rssi: -50,
        mdns: 'brewlogger',
        wifi_ssid: 'MyNetwork',
        ip: '192.168.1.100',
        total_heap: 262144,
        free_heap: 131072,
        wifi_setup: false,
        uptime_seconds: 3600,
        uptime_minutes: 60,
        uptime_hours: 1,
        uptime_days: 0,
        pid_mode: 'b',
        pid_state: 1,
        pid_state_string: 'Idle',
        pid_beer_temp: 20.5,
        pid_beer_temp_connected: true,
        pid_fridge_temp: 18.2,
        pid_fridge_temp_connected: true,
        pid_beer_target_temp: 20.0,
        pid_fridge_target_temp: 15.0,
        pid_temp_format: 'C',
        pid_cooling_actuator_active: false,
        pid_heating_actuator_active: true,
        pid_wait_time: 300,
        pid_time_since_cooling: 0,
        pid_time_since_heating: 120,
        pid_time_since_idle: 60,
        temperature_device: []
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useStatusStore()
      const result = await store.load()

      expect(result).toBe(true)
      expect(http.getJson).toHaveBeenCalledWith('api/status')
    })

    it('updates store with API response data', async () => {
      const mockData = {
        id: 'brewlogger-01',
        rssi: -45,
        mdns: 'brewlogger',
        wifi_ssid: 'MyNetwork',
        ip: '192.168.1.100',
        total_heap: 262144,
        free_heap: 131072,
        wifi_setup: true,
        uptime_seconds: 7200,
        uptime_minutes: 120,
        uptime_hours: 2,
        uptime_days: 0,
        pid_mode: 'f',
        pid_state: 2,
        pid_state_string: 'Running',
        pid_beer_temp: 19.8,
        pid_beer_temp_connected: true,
        pid_fridge_temp: 10.5,
        pid_fridge_temp_connected: true,
        pid_beer_target_temp: 20.0,
        pid_fridge_target_temp: 10.0,
        pid_temp_format: 'C',
        pid_cooling_actuator_active: true,
        pid_heating_actuator_active: false,
        pid_wait_time: 600,
        pid_time_since_cooling: 45,
        pid_time_since_heating: 0,
        pid_time_since_idle: 0,
        temperature_device: []
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useStatusStore()
      await store.load()

      expect(store.id).toBe('brewlogger-01')
      expect(store.rssi).toBe(-45)
      expect(store.mdns).toBe('brewlogger')
      expect(store.wifi_ssid).toBe('MyNetwork')
      expect(store.ip).toBe('192.168.1.100')
      expect(store.pid_mode).toBe('f')
      expect(store.pid_state_string).toBe('Running')
      expect(store.pid_beer_temp).toBe(19.8)
      expect(store.pid_cooling_actuator_active).toBe(true)
      expect(store.pid_heating_actuator_active).toBe(false)
    })

    it('converts heap memory from bytes to kilobytes and rounds', async () => {
      const mockData = {
        id: 'brewlogger-01',
        rssi: -50,
        mdns: 'brewlogger',
        wifi_ssid: 'MyNetwork',
        ip: '192.168.1.1',
        total_heap: 262144,
        free_heap: 131072,
        wifi_setup: false,
        uptime_seconds: 0,
        uptime_minutes: 0,
        uptime_hours: 0,
        uptime_days: 0,
        pid_mode: 'b',
        pid_state: 0,
        pid_state_string: 'Off',
        pid_beer_temp: 0,
        pid_beer_temp_connected: false,
        pid_fridge_temp: 0,
        pid_fridge_temp_connected: false,
        pid_beer_target_temp: 0,
        pid_fridge_target_temp: 0,
        pid_temp_format: 'C',
        pid_cooling_actuator_active: false,
        pid_heating_actuator_active: false,
        pid_wait_time: 0,
        pid_time_since_cooling: 0,
        pid_time_since_heating: 0,
        pid_time_since_idle: 0,
        temperature_device: []
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useStatusStore()
      await store.load()

      const expectedTotalHeap = Math.round(262144 / 1024).toFixed(0)
      const expectedFreeHeap = Math.round(131072 / 1024).toFixed(0)

      expect(store.total_heap.toString()).toBe(expectedTotalHeap)
      expect(store.free_heap.toString()).toBe(expectedFreeHeap)
    })

    it('handles temperature device array', async () => {
      const mockData = {
        id: 'brewlogger-01',
        rssi: -50,
        mdns: 'brewlogger',
        wifi_ssid: 'MyNetwork',
        ip: '192.168.1.1',
        total_heap: 262144,
        free_heap: 131072,
        wifi_setup: false,
        uptime_seconds: 0,
        uptime_minutes: 0,
        uptime_hours: 0,
        uptime_days: 0,
        pid_mode: 'b',
        pid_state: 0,
        pid_state_string: 'Off',
        pid_beer_temp: 0,
        pid_beer_temp_connected: false,
        pid_fridge_temp: 0,
        pid_fridge_temp_connected: false,
        pid_beer_target_temp: 0,
        pid_fridge_target_temp: 0,
        pid_temp_format: 'C',
        pid_cooling_actuator_active: false,
        pid_heating_actuator_active: false,
        pid_wait_time: 0,
        pid_time_since_cooling: 0,
        pid_time_since_heating: 0,
        pid_time_since_idle: 0,
        temperature_device: [{ device: 'Sensor1', temp: 19.5, source: 'BLE', type: 'Temp' }]
      }

      http.getJson.mockResolvedValue(mockData)

      const store = useStatusStore()
      await store.load()

      expect(store.temperature_device.length).toBe(1)
      expect(store.temperature_device[0].device).toBe('Sensor1')
      expect(store.temperature_device[0].temp).toBe(19.5)
    })

    it('handles API error gracefully', async () => {
      http.getJson.mockRejectedValue(new Error('Network error'))

      const store = useStatusStore()
      const result = await store.load()

      expect(result).toBe(false)
    })

    it('returns false on failure', async () => {
      http.getJson.mockRejectedValue(new Error('Failed to fetch status'))

      const store = useStatusStore()
      const result = await store.load()

      expect(result).toBe(false)
    })

    it('preserves existing state on error', async () => {
      const store = useStatusStore()
      store.id = 'brewlogger-01'
      store.rssi = -50

      http.getJson.mockRejectedValue(new Error('API failed'))

      await store.load()

      expect(store.id).toBe('brewlogger-01')
      expect(store.rssi).toBe(-50)
    })
  })

  describe('store state mutations', () => {
    it('allows updating device status', () => {
      const store = useStatusStore()
      store.id = 'brewlogger-02'
      store.rssi = -60
      expect(store.id).toBe('brewlogger-02')
      expect(store.rssi).toBe(-60)
    })

    it('allows updating PID state', () => {
      const store = useStatusStore()
      store.pid_mode = 'f'
      store.pid_state_string = 'Running'
      expect(store.pid_mode).toBe('f')
      expect(store.pid_state_string).toBe('Running')
    })

    it('allows updating temperature values', () => {
      const store = useStatusStore()
      store.pid_beer_temp = 19.5
      store.pid_fridge_temp = 10.2
      expect(store.pid_beer_temp).toBe(19.5)
      expect(store.pid_fridge_temp).toBe(10.2)
    })

    it('allows updating actuator states', () => {
      const store = useStatusStore()
      store.pid_cooling_actuator_active = true
      store.pid_heating_actuator_active = true
      expect(store.pid_cooling_actuator_active).toBe(true)
      expect(store.pid_heating_actuator_active).toBe(true)
    })

    it('allows updating connection state', () => {
      const store = useStatusStore()
      store.connected = false
      expect(store.connected).toBe(false)
    })

    it('allows updating sensor connection flags', () => {
      const store = useStatusStore()
      store.pid_beer_temp_connected = true
      store.pid_fridge_temp_connected = false
      expect(store.pid_beer_temp_connected).toBe(true)
      expect(store.pid_fridge_temp_connected).toBe(false)
    })
  })
})
