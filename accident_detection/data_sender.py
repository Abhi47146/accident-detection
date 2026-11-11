import httpx
from accident_detection.logger import Logger

logger = Logger('data_send')

headers = {"Content-Type": "application/json"}
client = httpx.Client(
    timeout=httpx.Timeout(5.0),  
    headers=headers
)

def send_to_api(vbv_file, api_url):
    try:
        response = client.post(api_url, json=vbv_file, headers=headers)
        response.raise_for_status()
        logger.info(f"Generated vbv_file {vbv_file}")

    except httpx.TimeoutException:
        logger.error("API request timed out")

    except httpx.ConnectError as e:
        logger.error(f"API unreachable: {e}")

    except httpx.HTTPStatusError as e:
        logger.error(f"API error {e.response.status_code}: {e.response.text}")

    except Exception:
        logger.exception("Unexpected error while sending vehicle data")
