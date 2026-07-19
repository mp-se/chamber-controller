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

    <template v-if="status">
      <!-- add fatal error messages here -->
    </template>

    <div v-if="status" class="container overflow-hidden text-center">
      <div class="row gy-4">
        <div class="col-md-6">
          <PidTemperatureFragment />
        </div>

        <div class="col-md-6">
          <PidControllerFragment />
        </div>

        <template v-if="global.feature.ble_sensor">
          <template v-for="device in status.temperature_device" :key="device.device">
            <div class="col-md-4">
              <BsCard
                :header="t('home.ble_device')"
                color="secondary"
                :title="device.device + ' (' + formatTime(device.update_time) + ')'"
              >
                <p class="text-center">
                  {{ t('home.temperature') }}: {{ formatTemp(device.temp) }}°{{ config.temp_format }}
                </p>

                <span class="badge bg-primary">{{ device.source }}</span
                >&nbsp;
                <span class="badge bg-primary">{{ device.type }}</span>
              </BsCard>
            </div>
          </template>
        </template>

        <div class="col-md-4">
          <BsCard :header="t('home.pid')" color="success" :title="t('home.controller')">
            <p class="text-center">
              {{ t('home.mode_label') }}
              {{
                status.pid_mode == 'b'
                  ? t('home.beer_constant')
                  : status.pid_mode == 'f'
                    ? t('home.fridge_constant')
                    : t('home.off')
              }}<br />
              {{ t('home.state_label') }}{{ status.pid_state_string }}<br />
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.pid')" color="success" :title="t('home.actuators')">
            <p class="text-center">
              {{ t('home.cooling_label') }}{{ status.pid_cooling_actuator_active ? t('home.active') : t('home.inactive') }}<br />
              {{ t('home.heating_label') }}{{ status.pid_heating_actuator_active ? t('home.active') : t('home.inactive') }}<br />
              {{ t('home.fan_label') }}{{ status.pid_fan_actuator_active ? t('home.active') : t('home.inactive') }}
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.pid')" color="success" :title="t('home.sensors')">
            <p class="text-center">
              {{ t('home.fridge_sensor_label') }}{{ status.fridge_sensor_id }}<br />
              {{ t('home.beer_sensor_label') }}{{ status.beer_sensor_id }}
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.pid')" color="success" :title="t('home.sensors')">
            <p class="text-center">
              {{ t('home.chamber_label') }}{{ formatTemp(status.pid_fridge_temp) }}°{{ config.temp_format }}<br />
              {{ t('home.beer_label') }}{{ formatTemp(status.pid_beer_temp) }}°{{ config.temp_format }}
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.measurement')" color="info" :title="t('home.wifi')">
            <p class="text-center">{{ status.rssi }} dBm - {{ status.wifi_ssid }}</p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.device')" color="info" :title="t('home.ip_address')">
            <p class="text-center">
              {{ status.ip }}
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.device')" color="info" :title="t('home.memory')">
            <p class="text-center">
              {{ t('home.memory_text', { free: status.free_heap, total: status.total_heap }) }}
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.device')" color="info" :title="t('home.software_version')">
            <p class="text-center">
              {{ t('home.software_version_text', { 
                  appVer: global.app_ver, 
                  appBuild: global.app_build, 
                  uiVersion: global.uiVersion, 
                  uiBuild: global.uiBuild 
                }) }}
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.device')" color="info" :title="t('home.platform')">
            <p class="text-center">
              <span class="badge bg-secondary">{{ global.platform }}</span>
            </p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.device')" color="info" :title="t('home.id')">
            <p class="text-center">{{ status.id }}</p>
          </BsCard>
        </div>

        <div class="col-md-4">
          <BsCard :header="t('home.device')" color="info" :title="t('home.uptime')">
            <p class="text-center">
              {{ t('home.uptime_text', { 
                  days: status.uptime_days, 
                  hours: status.uptime_hours, 
                  minutes: status.uptime_minutes, 
                  seconds: status.uptime_seconds 
                }) }}
            </p>
          </BsCard>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onBeforeMount, onBeforeUnmount } from 'vue'
import { status, config, global } from '@/modules/pinia'
import { tempToF } from '@mp-se/espframework-ui-components'
import PidControllerFragment from '@/fragments/PidControllerFragment.vue'
import PidTemperatureFragment from '@/fragments/PidTemperatureFragment.vue'
import { useI18n } from 'vue-i18n'

const { t } = useI18n()
const polling = ref(null)
const timer = ref(null)

function formatTime(t) {
  if (t < 60)
    // less than 1 min
    return new Number(t).toFixed(0) + 's'

  if (t < 60 * 60)
    // less than 1 hour
    return new Number(t / 60).toFixed(0) + 'm'

  if (t < 60 * 60 * 24)
    // less than 1 day
    return new Number(t / (60 * 60)).toFixed(0) + 'h'

  return new Number(t / (60 * 60 * 24)).toFixed(0) + 'd'
}

function formatTemp(t) {
  return config.temp_format === 'C' ? new Number(t).toFixed(2) : tempToF(new Number(t).toFixed(1))
}

function updateTimers() {
  // This is used to decrease timers so we can display every seconds even though status is polled every 5 seconds
  status.pid_time_since_idle += 1
  status.pid_wait_time -= 1
  status.pid_time_since_cooling += 1
  status.pid_time_since_heating += 1
}

async function refresh() {
  const success = await status.load()
  if (success) {
    // TODO: Add any logic you want update as part of status update
  }
}

onBeforeMount(() => {
  timer.value = setInterval(updateTimers, 1000)
  refresh()
  polling.value = setInterval(refresh, 4000)
})

onBeforeUnmount(() => {
  clearInterval(timer.value)
  clearInterval(polling.value)
})
</script>
<style></style>

