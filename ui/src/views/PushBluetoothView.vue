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
    <p class="h3">Push - Bluetooth</p>
    <hr />

    <template v-if="global.feature.ble">
      <form @submit.prevent="save" class="needs-validation" novalidate>
        <div class="row">
          <div class="col-md-12">
            <BsInputSwitch
              v-model="config.ble_push_enabled"
              label="Enable temperature sending over bluetooth"
              :disabled="global.disabled"
            />
          </div>
          <div class="col-md-12">
            <p></p>
            <p>Changing bluetooth settings might require a restart to function properly</p>
          </div>
        </div>
        <div class="row gy-2">
          <div class="col-md-12">
            <hr />
          </div>
          <div class="col-md-3">
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
              &nbsp;Save
            </button>
          </div>
        </div>
      </form>
    </template>
    <template v-else>
      <div class="row">
        <div class="col-md-12">
          <p>Bluetooth is not available on this platform</p>
        </div>
      </div>
    </template>
  </div>
</template>

<script setup>
import { validateCurrentForm } from '@mp-se/espframework-ui-components'
import { global, config } from '@/modules/pinia'

const save = async () => {
  if (!validateCurrentForm()) return

  global.clearMessages()
  await config.saveAll()
}
</script>
