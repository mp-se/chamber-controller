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

import { createPinia } from 'pinia'
import { useGlobalStore } from '@/modules/globalStore'
import { useStatusStore } from '@/modules/statusStore'
import { useConfigStore, saveConfigState as _saveConfigState, getConfigChanges as _getConfigChanges } from '@/modules/configStore'
import { logDebug } from '@mp-se/espframework-ui-components'

const piniaInstance = createPinia()

export default piniaInstance

const config = useConfigStore(piniaInstance)
const global = useGlobalStore(piniaInstance)
const status = useStatusStore(piniaInstance)

export { global, status, config }

config.$subscribe(() => {
  if (!global.initialized) return

  var changes = _getConfigChanges(config)
  logDebug('pinia:$subscribe()', 'State change on configStore', changes)

  if (JSON.stringify(changes).length > 2) {
    global.configChanged = true
    logDebug('pinia:$subscribe()', 'Changed properties:', changes)
  } else {
    global.configChanged = false
  }
})

// Wrappers so callers don't need to pass the config argument
export const saveConfigState = () => _saveConfigState(config)
export const getConfigChanges = () => _getConfigChanges(config)
