import './DebugConsole.css';
import Backend from '../Backend';
import { useState } from 'react';


function DebugConsole() {

    const [debugText, setdebugText] = useState('');

    const writeDebug = msg => {
        setdebugText(debugText + msg + '\n');
    }

    Backend.writeDebug = writeDebug;

    return <div className="debug-console">
        <textarea
            value={debugText}
            readOnly={true}
        />
    </div>
}

export default DebugConsole;