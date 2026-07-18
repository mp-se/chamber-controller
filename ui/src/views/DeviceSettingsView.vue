<!--
  Chamber Controller UI
  Copyright (c) 2021-2026 Magnus

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
-->

<template>
  <div class="container">
    <p></p>
    <p class="h2">Device - Settings</p>
    <hr />

    <BsMessage v-if="config.mdns === ''" dismissable="true" message="" alert="warning">
      You need to define a mdns name for the device
    </BsMessage>

    <form @submit.prevent="saveSettings" class="needs-validation" novalidate>
      <div class="row">
        <div class="col-md-12">
          <BsInputText
            v-model="config.mdns"
            maxlength="63"
            minlength="1"
            label="MDNS"
            help="Enter device name used on the network, the suffix .local will be added to this name"
            :badge="badge.deviceMdnsBadge()"
            width="8"
            :disabled="global.disabled"
          >
          </BsInputText>
        </div>

        <div class="col-md-12">
          <hr />
        </div>

        <div class="col-md-4">
          <BsInputRadio
            v-model="config.temp_format"
            :options="tempOptions"
            label="Temperature Format"
            :disabled="global.disabled"
          ></BsInputRadio>
        </div>

        <div class="col-md-4">
          <BsInputRadio
            v-model="config.dark_mode"
            :options="uiOptions"
            label="User Interface"
            :disabled="global.disabled"
          ></BsInputRadio>
        </div>

        <!-- 
        <div class="col-md-6">
          <BsInputNumber
            v-model="config.restart_interval"
            unit="minutes"
            label="Restart interval in minutes"
            min="30"
            max="1440"
            step="1"
            width="5"
            help="Interval when the device will restart to ensure stability (30-1440)"
            :disabled="global.disabled"
          ></BsInputNumber>
        </div>-->

        <div class="col-md-4">
          <BsSelect
            v-model="config.restart_interval"
            label="Restart interval"
            help="Interval when the device will restart to ensure stability"
            :options="restartOptions"
            :disabled="global.disabled"
          />
        </div>
      </div>

      <div class="row gy-2">
        <div class="col-md-12">
          <hr />
        </div>

        <div class="col-md-12">
          <button
            type="submit"
            class="btn btn-primary w-2"
            :disabled="global.disabled || !global.configChanged"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;Save</button
          >&nbsp;

          <button
            @click="restartDevice"
            type="button"
            class="btn btn-secondary"
            :disabled="global.disabled"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;Restart device</button
          >&nbsp;

          <button
            @click="factory"
            type="button"
            class="btn btn-secondary"
            :disabled="global.disabled"
          >
            <span
              class="spinner-border spinner-border-sm"
              role="status"
              aria-hidden="true"
              v-show="global.disabled"
            ></span>
            &nbsp;Restore factory defaults
          </button>
        </div>
      </div>
    </form>
  </div>
</template>

<script setup>
import { ref } from 'vue'
import { validateCurrentForm } from '@mp-se/espframework-ui-components'
import { global, config } from '@/modules/pinia'
import * as badge from '@/modules/badge'
import { logError } from '@mp-se/espframework-ui-components'

const tempOptions = ref([
  { label: 'Celsius °C', value: 'C' },
  { label: 'Fahrenheit °F', value: 'F' }
])

const uiOptions = ref([
  { label: 'Day mode', value: false },
  { label: 'Dark mode', value: true }
])

const restartOptions = ref([
  { label: '-- disabled --', value: 0 },
  { label: '30 minutes', value: 30 },
  { label: '1 hour', value: 60 },
  { label: '2 hours', value: 60 * 2 },
  { label: '4 hours', value: 60 * 4 },
  { label: '6 hours', value: 60 * 6 },
  { label: '12 hours', value: 60 * 12 },
  { label: '24 hours', value: 60 * 24 }
])

const factory = async () => {
  try {
    global.clearMessages()
    global.disabled = true

    const response = await fetch(global.baseURL + 'api/factory', {
      headers: { Authorization: global.token },
      signal: AbortSignal.timeout(global.fetchTimeout)
    })

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`)
    }

    const json = await response.json()

    if (json.success === true) {
      global.messageSuccess = json.message + ' Reloading page in 2 seconds...'

      // Use setTimeout with proper cleanup
      const reloadTimeout = setTimeout(() => {
        try {
          location.reload(true)
        } catch (error) {
          logError('DeviceSettingsView.factory.reload()', error)
          // Fallback reload
          window.location.reload()
        }
      }, 2000)

      // Clean up timeout on page unload
      window.addEventListener(
        'beforeunload',
        () => {
          clearTimeout(reloadTimeout)
        },
        { once: true }
      )
    } else {
      global.messageError = json.message || 'Factory restore failed'
    }
  } catch (err) {
    logError('DeviceSettingsView.factory()', err)
    global.messageError = 'Failed to perform factory restore: ' + (err.message || err)
  } finally {
    global.disabled = false
  }
}

const saveSettings = async () => {
  try {
    if (!validateCurrentForm()) return

    await config.saveAll()
  } catch (error) {
    logError('DeviceSettingsView.saveSettings()', error)
    global.messageError = 'Failed to save settings: ' + (error.message || error)
  }
}

const restartDevice = async () => {
  try {
    await config.restart()
  } catch (error) {
    logError('DeviceSettingsView.restartDevice()', error)
    global.messageError = 'Failed to restart device: ' + (error.message || error)
  }
}
</script>
