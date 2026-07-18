/*
 * Chamber Controller UI
 * Copyright (c) 2021-2026 Magnus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

import { defineStore } from 'pinia'
import { sharedHttpClient as http } from '@mp-se/espframework-ui-components'
import { logDebug, logInfo, logError } from '@mp-se/espframework-ui-components'

export const useStatusStore = defineStore('status', {
  state: () => {
    return {
      id: '',
      rssi: 0,
      mdns: '',
      wifi_ssid: '',
      ip: '',
      total_heap: 0,
      free_heap: 0,
      wifi_setup: false,

      uptime_seconds: 0,
      uptime_minutes: 0,
      uptime_hours: 0,
      uptime_days: 0,

      remote_control_active: false,

      pid_mode: '',
      pid_state: 0,
      pid_state_string: '',
      pid_beer_temp: 0,
      pid_beer_temp_connected: false,
      pid_fridge_temp: 0,
      pid_fridge_temp_connected: false,
      pid_beer_target_temp: 0,
      pid_fridge_target_temp: 0,
      pid_temp_format: '',
      pid_cooling_actuator_active: false,
      pid_heating_actuator_active: false,
      pid_wait_time: 0,
      pid_time_since_cooling: 0,
      pid_time_since_heating: 0,
      pid_time_since_idle: 0,

      fridge_sensor_id: '',
      beer_sensor_id: '',

      temperature_device: [],

      connected: true
    }
  },
  getters: {},
  actions: {
    async load() {
      try {
        logDebug('statusStore:load()', 'Fetching /api/status')
        const json = await http.getJson('api/status')
        logDebug('statusStore:load()', json)
        this.id = json.id
        this.rssi = json.rssi
        this.mdns = json.mdns
        this.wifi_ssid = json.wifi_ssid
        this.ip = json.ip
        this.total_heap = Math.round(json.total_heap / 1024).toFixed(0)
        this.free_heap = Math.round(json.free_heap / 1024).toFixed(0)
        this.wifi_setup = json.wifi_setup
        this.uptime_seconds = json.uptime_seconds
        this.uptime_minutes = json.uptime_minutes
        this.uptime_hours = json.uptime_hours
        this.uptime_days = json.uptime_days
        
        this.remote_control_active = json.remote_control_active

        this.pid_mode = json.pid_mode
        this.pid_state = json.pid_state
        this.pid_state_string = json.pid_state_string
        this.pid_beer_temp = json.pid_beer_temp
        this.pid_beer_temp_connected = json.pid_beer_temp_connected
        this.pid_fridge_temp = json.pid_fridge_temp
        this.pid_fridge_temp_connected = json.pid_fridge_temp_connected
        this.pid_beer_target_temp = json.pid_beer_target_temp
        this.pid_fridge_target_temp = json.pid_fridge_target_temp
        this.pid_temp_format = json.pid_temp_format
        this.pid_cooling_actuator_active = json.pid_cooling_actuator_active
        this.pid_heating_actuator_active = json.pid_heating_actuator_active
        this.pid_wait_time = json.pid_wait_time
        this.pid_time_since_cooling = json.pid_time_since_cooling
        this.pid_time_since_heating = json.pid_time_since_heating
        this.pid_time_since_idle = json.pid_time_since_idle

        this.fridge_sensor_id = json.fridge_sensor_id
        this.beer_sensor_id = json.beer_sensor_id

        this.temperature_device = json.temperature_device

        logInfo('statusStore:load()', 'Fetching /api/status completed')
        return true
      } catch (err) {
        logError('statusStore:load()', 'Fetching /api/status failed', err)
        return false
      }
    }
  }
})
